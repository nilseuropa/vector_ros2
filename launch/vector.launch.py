import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    TextSubstitution,
    Command,
    PathJoinSubstitution,
)
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    venv_site_packages = LaunchConfiguration("venv_site_packages")
    use_robot_state_publisher = LaunchConfiguration("use_robot_state_publisher")
    model = LaunchConfiguration("model")
    default_venv = os.environ.get("VECTOR_VENV_SITE_PACKAGES", "")
    if not default_venv:
        default_venv = "/home/nils/vector_ws/.vector/lib/python3.12/site-packages"
    default_model = PathJoinSubstitution(
        [FindPackageShare("vector_ros2"), "urdf", "vector.xacro"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("venv_site_packages", default_value=default_venv),
            DeclareLaunchArgument("use_robot_state_publisher", default_value="true"),
            DeclareLaunchArgument("model", default_value=default_model),
            SetEnvironmentVariable(
                name="PYTHONPATH",
                value=[
                    EnvironmentVariable("PYTHONPATH", default_value=""),
                    TextSubstitution(text=":"),
                    venv_site_packages,
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                condition=IfCondition(use_robot_state_publisher),
                parameters=[
                    {
                        "robot_description": Command(
                            [FindExecutable(name="xacro"), " ", model]
                        )
                    }
                ],
            ),
            Node(
                package="vector_ros2",
                executable="vector_driver",
                name="vector_driver",
                output="screen",
                parameters=[
                    {
                        "serial": "",
                        "ip": "",
                        "state_hz": 10.0,
                        "battery_hz": 1.0,
                        "camera_hz": 5.0,
                        "cmd_vel_timeout_sec": 0.5,
                        "enable_camera": True,
                        "enable_face_detection": False,
                        "enable_nav_map_feed": False,
                        "enable_custom_object_detection": False,
                        "enable_audio_feed": False,
                        "frame_odom": "odom",
                        "frame_base": "base_link",
                        "frame_footprint": "base_footprint",
                        "wheel_track_mm": 50.0,
                        "max_wheel_speed_mmps": 200.0,
                        "publish_tf": True,
                        "joint_head_name": "base_to_head",
                        "joint_lift_name": "base_to_lift",
                        "lift_use_angle": True,
                        "lift_height_to_angle_scale": -13.7931034483,
                        "lift_height_to_angle_offset": 0.4413793103,
                        "lift_height_min_m": 0.032,
                        "lift_height_max_m": 0.09,
                        "imu_accel_scale": 0.001,
                        "imu_gyro_scale": 1.0,
                        "head_angle_scale": -1.0,
                        "head_angle_offset": 0.0,
                    }
                ],
            )
        ]
    )
