# vector_ros2

ROS 2 (Jazzy) wrapper for Anki Vector running OSKR + wire-pod.

## What it does

Publishes:
- `pose` (geometry_msgs/PoseStamped)
- `odom` (nav_msgs/Odometry)
- `imu` (sensor_msgs/Imu)
- `battery` (sensor_msgs/BatteryState)
- `proximity/range` (sensor_msgs/Range)
- `touch` (std_msgs/Bool)
- `touch/raw` (std_msgs/Float32)
- `cliff_detected` (std_msgs/Bool)
- `joint_states` (sensor_msgs/JointState) for head and lift (URDF joint names)
- `head_angle` (std_msgs/Float32)
- `lift_height` (std_msgs/Float32)
- `camera/image_raw` (sensor_msgs/Image)

Subscribes:
- `cmd_vel` (geometry_msgs/Twist)
- `head_angle_cmd` (std_msgs/Float32)
- `lift_height_cmd` (std_msgs/Float32)

Services:
- `start_exploration` (std_srvs/Trigger)
- `fetch_cube` (std_srvs/Trigger)
- `go_home` (std_srvs/Trigger)
- `drive_off_charger` (std_srvs/Trigger)
- `play_animation` (vector_ros2/srv/PlayAnimationTrigger)
- `say_text` (vector_ros2/srv/SayText)

## Setup

1) Create and activate a venv with ROS site-packages:

```bash
python3 -m venv --system-site-packages /home/nils/vector_ws/.vector
source /home/nils/vector_ws/.vector/bin/activate
python3 -m pip install --upgrade pip
```

2) Install the wire-pod SDK fork:

```bash
python3 -m pip install wirepod_vector_sdk
```

3) Configure SDK creds (this writes `~/.anki_vector`):

```bash
python3 -m anki_vector.configure
```

4) Build the ROS 2 package:

```bash
cd /home/nils/vector_ws
colcon build --packages-select vector_ros2
source install/setup.bash
```

5) Run:

```bash
ros2 launch vector_ros2 vector.launch.py
```

## Notes

- `cmd_vel` is converted to wheel speeds using `wheel_track_mm`. Tune if turning feels off.
- `max_wheel_speed_mmps` limits wheel speed (set to 0 to disable limit).
- `head_angle_cmd` is radians.
- `lift_height` topic is meters. `lift_height_cmd` is also meters; the driver maps it to the SDK’s
  normalized 0.0–1.0 range using `lift_height_min_m`/`lift_height_max_m`.
- IMU values are scaled by `imu_accel_scale` and `imu_gyro_scale` before publishing.
- If you use a venv for the SDK, set `venv_site_packages` in the launch file or export
  `VECTOR_VENV_SITE_PACKAGES` to point at the venv site-packages. The launch file
  appends this to `PYTHONPATH` so ROS 2 can still find the package metadata.
- The Vector URDF/xacro and meshes are included in this package and launched with
  `robot_state_publisher` by default.

## Parameters

- `serial` (string): optional Vector serial (recommended if you have multiple)
- `ip` (string): optional Vector IP
- `state_hz` (float): publish rate for pose/imu/etc
- `battery_hz` (float): publish rate for battery
- `camera_hz` (float): publish rate for camera
- `cmd_vel_timeout_sec` (float): stop motors if no cmd_vel for this duration
- `enable_camera` (bool)
- `enable_face_detection` (bool)
- `enable_nav_map_feed` (bool)
- `enable_custom_object_detection` (bool)
- `enable_audio_feed` (bool)
- `frame_odom` (string)
- `frame_base` (string)
- `frame_footprint` (string): child frame for odom/TF (default `base_footprint`)
- `wheel_track_mm` (float)
- `max_wheel_speed_mmps` (float)
- `publish_tf` (bool): publish `odom` → `base_link` transform
- `use_robot_state_publisher` (bool): launch `robot_state_publisher` (default true)
- `model` (string): path to the xacro model (default vector_ros2/urdf/vector.xacro)
- `joint_head_name` (string): URDF joint for head (default `base_to_head`)
- `joint_lift_name` (string): URDF joint for lift (default `base_to_lift`)
- `lift_use_angle` (bool): use SDK `lift_angle_rad` if available
- `lift_height_to_angle_scale` (float): scale meters → radians when lift angle isn't available
- `lift_height_to_angle_offset` (float): offset radians when lift angle isn't available
- `lift_height_min_m` (float): minimum lift height (meters) for command mapping (default 0.032)
- `lift_height_max_m` (float): maximum lift height (meters) for command mapping (default 0.09)
- `imu_accel_scale` (float): multiply accel values from SDK (default 0.001 to convert mm/s^2 → m/s^2)
- `imu_gyro_scale` (float): multiply gyro values from SDK (default 1.0)
- `head_angle_scale` (float): scale head angle from SDK before publishing joint state
- `head_angle_offset` (float): offset head angle (radians) before publishing joint state

## Services

`play_animation` expects an animation trigger name:

```bash
ros2 service call /play_animation vector_ros2/srv/PlayAnimationTrigger "{name: GreetAfterLongTime}"
```

List available animation triggers via the SDK:

```bash
python - <<'PY'
import anki_vector

with anki_vector.Robot() as robot:
    for name in robot.anim.anim_trigger_list:
        print(name)
PY
```

## Next steps

- Add face/object detection topics.
- Add audio input/output topics.
- Publish TF tree.
- Add diagnostics for SDK connection state.
