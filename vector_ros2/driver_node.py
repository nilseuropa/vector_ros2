import threading
import time
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Image, Imu, JointState, Range
from std_msgs.msg import Bool, Float32, String
from tf2_ros import TransformBroadcaster

try:
    import anki_vector
    from anki_vector import util
except Exception as exc:  # pragma: no cover - import guard for runtime
    anki_vector = None
    util = None
    _import_error = exc
else:
    _import_error = None


class VectorRos2Driver(Node):
    def __init__(self):
        super().__init__("vector_driver")

        self.declare_parameter("serial", "")
        self.declare_parameter("ip", "")
        self.declare_parameter("state_hz", 10.0)
        self.declare_parameter("battery_hz", 1.0)
        self.declare_parameter("camera_hz", 5.0)
        self.declare_parameter("cmd_vel_timeout_sec", 0.5)
        self.declare_parameter("enable_camera", True)
        self.declare_parameter("enable_face_detection", False)
        self.declare_parameter("enable_nav_map_feed", False)
        self.declare_parameter("enable_custom_object_detection", False)
        self.declare_parameter("enable_audio_feed", False)
        self.declare_parameter("frame_odom", "odom")
        self.declare_parameter("frame_base", "base_link")
        self.declare_parameter("frame_footprint", "base_footprint")
        self.declare_parameter("wheel_track_mm", 50.0)
        self.declare_parameter("max_wheel_speed_mmps", 200.0)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("joint_head_name", "base_to_head")
        self.declare_parameter("joint_lift_name", "base_to_lift")
        self.declare_parameter("lift_use_angle", True)
        self.declare_parameter("lift_height_to_angle_scale", 1.0)
        self.declare_parameter("lift_height_to_angle_offset", 0.0)
        self.declare_parameter("lift_height_min_m", 0.032)
        self.declare_parameter("lift_height_max_m", 0.09)
        self.declare_parameter("imu_accel_scale", 0.001)
        self.declare_parameter("imu_gyro_scale", 1.0)
        self.declare_parameter("head_angle_scale", 1.0)
        self.declare_parameter("head_angle_offset", 0.0)

        if _import_error is not None:
            raise RuntimeError(
                "anki_vector import failed. Install wirepod_vector_sdk. "
                f"Original error: {_import_error}"
            )

        self._robot = None
        self._robot_lock = threading.Lock()
        self._cmd_executor = ThreadPoolExecutor(max_workers=1)
        self._last_cmd_time = None
        self._last_image_id = None
        self._warned_lift_angle = False
        self._tf_broadcaster = TransformBroadcaster(self)

        self._setup_publishers()
        self._setup_subscribers()
        self._connect_robot()
        self._setup_timers()

    def _setup_publishers(self):
        self.pub_pose = self.create_publisher(PoseStamped, "pose", 10)
        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.pub_imu = self.create_publisher(Imu, "imu", 10)
        self.pub_battery = self.create_publisher(BatteryState, "battery", 10)
        self.pub_range = self.create_publisher(Range, "proximity/range", 10)
        self.pub_touch = self.create_publisher(Bool, "touch", 10)
        self.pub_touch_raw = self.create_publisher(Float32, "touch/raw", 10)
        self.pub_cliff = self.create_publisher(Bool, "cliff_detected", 10)
        self.pub_joint = self.create_publisher(JointState, "joint_states", 10)
        self.pub_head_angle = self.create_publisher(Float32, "head_angle", 10)
        self.pub_lift_height = self.create_publisher(Float32, "lift_height", 10)
        self.pub_image = self.create_publisher(Image, "camera/image_raw", 10)

    def _setup_subscribers(self):
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Float32, "head_angle_cmd", self._on_head_angle, 10)
        self.create_subscription(Float32, "lift_height_cmd", self._on_lift_height, 10)
        self.create_subscription(String, "say_text", self._on_say_text, 10)
        self.create_subscription(String, "play_animation_trigger", self._on_animation, 10)
        self.create_subscription(Bool, "drive_charger", self._on_drive_charger, 10)

    def _connect_robot(self):
        serial = self.get_parameter("serial").get_parameter_value().string_value.strip()
        ip = self.get_parameter("ip").get_parameter_value().string_value.strip()
        enable_camera = self.get_parameter("enable_camera").get_parameter_value().bool_value
        enable_face_detection = (
            self.get_parameter("enable_face_detection").get_parameter_value().bool_value
        )
        enable_nav_map_feed = (
            self.get_parameter("enable_nav_map_feed").get_parameter_value().bool_value
        )
        enable_custom_object_detection = (
            self.get_parameter("enable_custom_object_detection").get_parameter_value().bool_value
        )
        enable_audio_feed = (
            self.get_parameter("enable_audio_feed").get_parameter_value().bool_value
        )

        kwargs = {}
        if serial:
            kwargs["serial"] = serial
        if ip:
            kwargs["ip"] = ip
        if enable_face_detection:
            kwargs["enable_face_detection"] = True
        if enable_nav_map_feed:
            kwargs["enable_nav_map_feed"] = True
        if enable_custom_object_detection:
            kwargs["enable_custom_object_detection"] = True
        if enable_audio_feed:
            kwargs["enable_audio_feed"] = True

        self.get_logger().info("Connecting to Vector...")
        self._robot = anki_vector.Robot(**kwargs)
        self._robot.connect()
        self.get_logger().info("Vector connected.")

        if enable_camera:
            self._robot.camera.init_camera_feed()

    def _setup_timers(self):
        state_hz = self.get_parameter("state_hz").get_parameter_value().double_value
        battery_hz = self.get_parameter("battery_hz").get_parameter_value().double_value
        camera_hz = self.get_parameter("camera_hz").get_parameter_value().double_value

        if state_hz > 0:
            self.create_timer(1.0 / state_hz, self._publish_state)
        if battery_hz > 0:
            self.create_timer(1.0 / battery_hz, self._publish_battery)
        if camera_hz > 0:
            self.create_timer(1.0 / camera_hz, self._publish_camera)

    def _stamp(self):
        now = self.get_clock().now().to_msg()
        return now

    def _get_pose_quat(self, pose):
        rot = getattr(pose, "rotation", None)
        if rot is not None:
            q0 = getattr(rot, "q0", 1.0)
            q1 = getattr(rot, "q1", 0.0)
            q2 = getattr(rot, "q2", 0.0)
            q3 = getattr(rot, "q3", 0.0)
        else:
            q0 = getattr(pose, "q0", 1.0)
            q1 = getattr(pose, "q1", 0.0)
            q2 = getattr(pose, "q2", 0.0)
            q3 = getattr(pose, "q3", 0.0)
        # SDK uses q0 as w.
        return q0, q1, q2, q3


    def _has_subscribers(self, publisher):
        return publisher.get_subscription_count() > 0

    def _publish_state(self):
        if self._robot is None:
            return
        if not self._robot_lock.acquire(blocking=False):
            return

        publish_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value
        need_imu = self._has_subscribers(self.pub_imu)
        need_pose = (
            publish_tf
            or need_imu
            or self._has_subscribers(self.pub_pose)
            or self._has_subscribers(self.pub_odom)
        )
        need_proximity = self._has_subscribers(self.pub_range)
        need_touch = self._has_subscribers(self.pub_touch) or self._has_subscribers(
            self.pub_touch_raw
        )
        need_status = self._has_subscribers(self.pub_cliff)
        need_joint = True
        need_head_angle = need_joint or self._has_subscribers(self.pub_head_angle)
        need_lift = need_joint or self._has_subscribers(self.pub_lift_height)
        lift_use_angle = self.get_parameter("lift_use_angle").get_parameter_value().bool_value

        try:
            pose = self._robot.pose if need_pose else None
            accel = self._robot.accel if need_imu else None
            gyro = self._robot.gyro if need_imu else None
            head_angle = (
                getattr(self._robot, "head_angle_rad", None) if need_head_angle else None
            )
            lift_height = (
                getattr(self._robot, "lift_height_mm", None) if need_lift else None
            )
            lift_angle = (
                getattr(self._robot, "lift_angle_rad", None)
                if need_joint and lift_use_angle
                else None
            )
            touch = self._robot.touch.last_sensor_reading if need_touch else None
            proximity = (
                self._robot.proximity.last_sensor_reading if need_proximity else None
            )
            status = getattr(self._robot, "status", None) if need_status else None
        finally:
            self._robot_lock.release()

        stamp = self._stamp()
        frame_odom = self.get_parameter("frame_odom").get_parameter_value().string_value
        frame_base = self.get_parameter("frame_base").get_parameter_value().string_value
        frame_footprint = (
            self.get_parameter("frame_footprint").get_parameter_value().string_value
        )
        if pose is not None and need_pose:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = frame_odom
            q0, q1, q2, q3 = self._get_pose_quat(pose)
            pose_msg.pose.position.x = pose.position.x / 1000.0
            pose_msg.pose.position.y = pose.position.y / 1000.0
            pose_msg.pose.position.z = pose.position.z / 1000.0
            pose_msg.pose.orientation.w = q0
            pose_msg.pose.orientation.x = q1
            pose_msg.pose.orientation.y = q2
            pose_msg.pose.orientation.z = q3
            if self._has_subscribers(self.pub_pose):
                self.pub_pose.publish(pose_msg)

            if self._has_subscribers(self.pub_odom):
                odom = Odometry()
                odom.header = pose_msg.header
                odom.child_frame_id = frame_footprint
                odom.pose.pose = pose_msg.pose
                self.pub_odom.publish(odom)

            if publish_tf:
                tf_msg = TransformStamped()
                tf_msg.header.stamp = stamp
                tf_msg.header.frame_id = frame_odom
                tf_msg.child_frame_id = frame_footprint
                tf_msg.transform.translation.x = pose_msg.pose.position.x
                tf_msg.transform.translation.y = pose_msg.pose.position.y
                tf_msg.transform.translation.z = pose_msg.pose.position.z
                tf_msg.transform.rotation = pose_msg.pose.orientation
                self._tf_broadcaster.sendTransform(tf_msg)

        if need_imu:
            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = frame_base
            if pose is not None:
                q0, q1, q2, q3 = self._get_pose_quat(pose)
                imu_msg.orientation.w = q0
                imu_msg.orientation.x = q1
                imu_msg.orientation.y = q2
                imu_msg.orientation.z = q3
            accel_scale = (
                self.get_parameter("imu_accel_scale")
                .get_parameter_value()
                .double_value
            )
            gyro_scale = (
                self.get_parameter("imu_gyro_scale")
                .get_parameter_value()
                .double_value
            )
            if accel is not None:
                imu_msg.linear_acceleration.x = float(accel.x) * accel_scale
                imu_msg.linear_acceleration.y = float(accel.y) * accel_scale
                imu_msg.linear_acceleration.z = float(accel.z) * accel_scale
            if gyro is not None:
                imu_msg.angular_velocity.x = float(gyro.x) * gyro_scale
                imu_msg.angular_velocity.y = float(gyro.y) * gyro_scale
                imu_msg.angular_velocity.z = float(gyro.z) * gyro_scale
            self.pub_imu.publish(imu_msg)

        if need_proximity and proximity is not None:
            range_msg = Range()
            range_msg.header.stamp = stamp
            range_msg.header.frame_id = frame_base
            range_msg.radiation_type = Range.INFRARED
            range_msg.field_of_view = 0.436
            range_msg.min_range = 0.03
            range_msg.max_range = 1.2
            if hasattr(proximity, "distance") and proximity.distance is not None:
                dist_mm = getattr(proximity.distance, "distance_mm", None)
                if dist_mm is not None:
                    range_msg.range = dist_mm / 1000.0
            self.pub_range.publish(range_msg)

        if need_touch and touch is not None:
            touch_msg = Bool()
            touch_msg.data = bool(getattr(touch, "is_being_touched", False))
            if self._has_subscribers(self.pub_touch):
                self.pub_touch.publish(touch_msg)

            touch_raw = Float32()
            touch_raw.data = float(getattr(touch, "raw_touch_value", 0.0))
            if self._has_subscribers(self.pub_touch_raw):
                self.pub_touch_raw.publish(touch_raw)

        if need_status and status is not None:
            cliff_msg = Bool()
            cliff_msg.data = bool(getattr(status, "is_cliff_detected", False))
            self.pub_cliff.publish(cliff_msg)

        joint = JointState()
        joint.header.stamp = stamp
        joint.name = [
            self.get_parameter("joint_head_name").get_parameter_value().string_value,
            self.get_parameter("joint_lift_name").get_parameter_value().string_value,
        ]

        head_scale = self.get_parameter("head_angle_scale").get_parameter_value().double_value
        head_offset = self.get_parameter("head_angle_offset").get_parameter_value().double_value
        head_pos = 0.0
        if head_angle is not None:
            head_pos = float(head_angle) * head_scale + head_offset
        lift_pos = 0.0
        if lift_use_angle and lift_angle is not None:
            lift_pos = float(lift_angle)
        elif lift_height is not None:
            scale = (
                self.get_parameter("lift_height_to_angle_scale")
                .get_parameter_value()
                .double_value
            )
            offset = (
                self.get_parameter("lift_height_to_angle_offset")
                .get_parameter_value()
                .double_value
            )
            lift_pos = (float(lift_height) / 1000.0) * scale + offset
            if lift_use_angle and not self._warned_lift_angle:
                self.get_logger().warn(
                    "lift_angle_rad not available; using lift_height_mm with "
                    "lift_height_to_angle_* params for joint state."
                )
                self._warned_lift_angle = True
        joint.position = [head_pos, lift_pos]
        self.pub_joint.publish(joint)

        if head_angle is not None and self._has_subscribers(self.pub_head_angle):
            head_msg = Float32()
            head_msg.data = float(head_angle)
            self.pub_head_angle.publish(head_msg)
        if lift_height is not None and self._has_subscribers(self.pub_lift_height):
            lift_msg = Float32()
            lift_msg.data = float(lift_height) / 1000.0
            self.pub_lift_height.publish(lift_msg)

        self._handle_cmd_timeout()

    def _publish_battery(self):
        if self._robot is None:
            return
        if not self._has_subscribers(self.pub_battery):
            return
        if not self._robot_lock.acquire(blocking=False):
            return
        try:
            batt = self._robot.get_battery_state()
        except Exception:
            return
        finally:
            self._robot_lock.release()
        msg = BatteryState()
        msg.header.stamp = self._stamp()
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.present = True
        for attr in ("battery_level", "battery_voltage", "battery_current"):
            if hasattr(batt, attr):
                value = getattr(batt, attr)
                if attr == "battery_level":
                    msg.percentage = float(value)
                if attr == "battery_voltage":
                    msg.voltage = float(value)
                if attr == "battery_current":
                    msg.current = float(value)
        self.pub_battery.publish(msg)

    def _publish_camera(self):
        if self._robot is None:
            return
        if not self.get_parameter("enable_camera").get_parameter_value().bool_value:
            return
        if not self._has_subscribers(self.pub_image):
            return
        if not self._robot_lock.acquire(blocking=False):
            return
        try:
            image = self._robot.camera.latest_image
        finally:
            self._robot_lock.release()
        if image is None:
            return
        image_id = getattr(image, "image_id", None)
        if image_id is not None and image_id == self._last_image_id:
            return
        self._last_image_id = image_id

        raw = image.raw_image
        if raw.mode != "RGB":
            raw = raw.convert("RGB")
        msg = Image()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self.get_parameter("frame_base").get_parameter_value().string_value
        msg.height = raw.height
        msg.width = raw.width
        msg.encoding = "rgb8"
        msg.step = raw.width * 3
        msg.data = raw.tobytes()
        self.pub_image.publish(msg)

    def _handle_cmd_timeout(self):
        timeout = self.get_parameter("cmd_vel_timeout_sec").get_parameter_value().double_value
        if timeout <= 0:
            return
        if self._last_cmd_time is None:
            return
        now = self.get_clock().now()
        elapsed = (now - self._last_cmd_time).nanoseconds / 1e9
        if elapsed <= timeout:
            return
        self._last_cmd_time = None
        self._cmd_executor.submit(self._set_wheel_speeds, 0.0, 0.0)

    def _on_cmd_vel(self, msg: Twist):
        wheel_track_mm = self.get_parameter("wheel_track_mm").get_parameter_value().double_value
        max_speed = self.get_parameter("max_wheel_speed_mmps").get_parameter_value().double_value
        linear_mps = msg.linear.x
        angular_rps = msg.angular.z
        left = (linear_mps - angular_rps * (wheel_track_mm / 1000.0) / 2.0) * 1000.0
        right = (linear_mps + angular_rps * (wheel_track_mm / 1000.0) / 2.0) * 1000.0
        if max_speed > 0:
            left = max(-max_speed, min(max_speed, left))
            right = max(-max_speed, min(max_speed, right))
        self._last_cmd_time = self.get_clock().now()
        self._cmd_executor.submit(self._set_wheel_speeds, left, right)

    def _set_wheel_speeds(self, left_mmps: float, right_mmps: float):
        if self._robot is None:
            return
        with self._robot_lock:
            try:
                self._robot.motors.set_wheel_motors(left_mmps, right_mmps)
            except Exception:
                pass

    def _on_head_angle(self, msg: Float32):
        if self._robot is None:
            return
        angle = msg.data
        with self._robot_lock:
            try:
                if hasattr(util, "radians"):
                    self._robot.behavior.set_head_angle(util.radians(angle))
                else:
                    self._robot.behavior.set_head_angle(util.Angle(radians=angle))
            except Exception:
                pass

    def _on_lift_height(self, msg: Float32):
        if self._robot is None:
            return
        height = msg.data
        min_m = self.get_parameter("lift_height_min_m").get_parameter_value().double_value
        max_m = self.get_parameter("lift_height_max_m").get_parameter_value().double_value
        if max_m <= min_m:
            norm = 0.0
        else:
            norm = (height - min_m) / (max_m - min_m)
        if norm < 0.0:
            norm = 0.0
        if norm > 1.0:
            norm = 1.0
        with self._robot_lock:
            try:
                self._robot.behavior.set_lift_height(norm)
            except Exception:
                pass

    def _on_say_text(self, msg: String):
        if self._robot is None:
            return
        text = msg.data.strip()
        if not text:
            return
        with self._robot_lock:
            try:
                self._robot.behavior.say_text(text)
            except Exception:
                pass

    def _on_animation(self, msg: String):
        if self._robot is None:
            return
        trigger = msg.data.strip()
        if not trigger:
            return
        with self._robot_lock:
            try:
                self._robot.anim.play_animation_trigger(trigger)
            except Exception:
                pass

    def _on_drive_charger(self, msg: Bool):
        if self._robot is None:
            return
        with self._robot_lock:
            try:
                if msg.data:
                    self._robot.behavior.drive_on_charger()
                else:
                    self._robot.behavior.drive_off_charger()
            except Exception:
                pass

    def destroy_node(self):
        try:
            if self._robot is not None:
                self._robot.disconnect()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = VectorRos2Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
