"""Unified, explicitly-disarmed physical stack for one drone."""

import os
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _validate(context):
    dry_run = _as_bool(LaunchConfiguration("dry_run").perform(context))
    if _as_bool(LaunchConfiguration("auto_arm").perform(context)):
        raise RuntimeError("auto_arm is forbidden; use the explicit /<drone_id>/arm service")
    if _as_bool(LaunchConfiguration("enable_legacy_esp32_bridge").perform(context)):
        raise RuntimeError("legacy CSV ESP32 bridge is incompatible with current binary firmware")
    for name in ("esp_device_id", "sender_id"):
        if len(LaunchConfiguration(name).perform(context).encode("ascii")) != 1:
            raise RuntimeError(f"{name} must be exactly one ASCII character")
    # The current SLAM package owns these conventional frame names internally.
    # Expose them for deployment truthfulness, but reject a silent partial rename.
    expected = {"map_frame": "map", "odom_frame": "odom", "base_frame": "base_link"}
    for name, value in expected.items():
        if LaunchConfiguration(name).perform(context) != value:
            raise RuntimeError(f"current SLAM configuration requires {name}:={value}")
    if float(LaunchConfiguration("apriltag_size_m").perform(context)) <= 0:
        raise RuntimeError("apriltag_size_m must be measured and positive")
    max_flight_time = float(LaunchConfiguration("max_flight_time_s").perform(context))
    if not math.isfinite(max_flight_time) or max_flight_time < 0 or (
        not dry_run and max_flight_time <= 0
    ):
        raise RuntimeError(
            "max_flight_time_s must be positive on hardware; zero is dry-run only"
        )
    return []


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("drone_id", default_value="d1"),
        DeclareLaunchArgument("num_drones", default_value="1"),
        DeclareLaunchArgument("role_coordinator_id", default_value="d1"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("esp_host", default_value="192.168.1.240"),
        DeclareLaunchArgument("esp_port", default_value="70"),
        DeclareLaunchArgument("esp_device_id", default_value="U"),
        DeclareLaunchArgument("sender_id", default_value="P"),
        DeclareLaunchArgument("transport", default_value="tcp"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("serial_baud", default_value="115200"),
        DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
        DeclareLaunchArgument("image_width", default_value="2560"),
        DeclareLaunchArgument("image_height", default_value="960"),
        DeclareLaunchArgument("framerate", default_value="10.0"),
        DeclareLaunchArgument("sensor_serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("sensor_serial_baud", default_value="115200"),
        DeclareLaunchArgument("enable_legacy_esp32_bridge", default_value="false"),
        DeclareLaunchArgument("enable_perception", default_value="true"),
        DeclareLaunchArgument("apriltag_size_m", default_value="0.0381"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("arena_width", default_value="91.44"),
        DeclareLaunchArgument("arena_height", default_value="24.38"),
        DeclareLaunchArgument(
            "arena_map_aligned", default_value="false",
            description="Set true only after map origin/axes are surveyed to the arena frame",
        ),
        DeclareLaunchArgument(
            "require_battery_valid", default_value="true",
            description="Disable only with a documented independent battery monitor/cutoff",
        ),
        DeclareLaunchArgument(
            "max_flight_time_s", default_value="420.0",
            description="Hard arm-to-controlled-LAND limit; zero is allowed only in dry-run",
        ),
    ]
    lc = LaunchConfiguration
    slam_launch = os.path.join(get_package_share_directory("slam"), "launch", "slam.launch.py")
    perception_config = os.path.join(
        get_package_share_directory("waar_perception"), "config", "apriltag.yaml"
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            "video_device": lc("camera_device"),
            "serial_port": lc("sensor_serial_port"),
            "serial_baudrate": lc("sensor_serial_baud"),
            "image_width": lc("image_width"),
            "image_height": lc("image_height"),
            "framerate": lc("framerate"),
            "use_legacy_esp32_bridge": "false",
            "enable_navsat": "false",
        }.items(),
    )
    bridge = Node(
        package="drone_hardware_bridge", executable="hardware_bridge_node",
        name="drone_hardware_bridge", output="screen",
        parameters=[{
            "drone_id": lc("drone_id"),
            "dry_run": ParameterValue(lc("dry_run"), value_type=bool),
            "auto_arm": ParameterValue(lc("auto_arm"), value_type=bool),
            "esp_host": lc("esp_host"),
            "esp_port": ParameterValue(lc("esp_port"), value_type=int),
            "esp_device_id": lc("esp_device_id"), "sender_id": lc("sender_id"),
            "transport": lc("transport"), "serial_port": lc("serial_port"),
            "serial_baud": ParameterValue(lc("serial_baud"), value_type=int),
            "map_frame": lc("map_frame"), "odom_frame": lc("odom_frame"),
            "base_frame": lc("base_frame"),
            "arena_width": ParameterValue(lc("arena_width"), value_type=float),
            "arena_height": ParameterValue(lc("arena_height"), value_type=float),
            "arena_map_aligned": ParameterValue(lc("arena_map_aligned"), value_type=bool),
            "require_battery_valid": ParameterValue(lc("require_battery_valid"), value_type=bool),
            "max_flight_time_s": ParameterValue(lc("max_flight_time_s"), value_type=float),
        }],
    )
    perception = Node(
        package="waar_perception", executable="apriltag_mine_node",
        name="apriltag_mine_node", output="screen",
        condition=IfCondition(lc("enable_perception")),
        parameters=[perception_config, {
            "drone_id": lc("drone_id"), "map_frame": lc("map_frame"),
            "tag_size_m": ParameterValue(lc("apriltag_size_m"), value_type=float),
            "arena_x_max": ParameterValue(lc("arena_width"), value_type=float),
            "arena_y_max": ParameterValue(lc("arena_height"), value_type=float),
        }],
    )
    common = {"drone_id": lc("drone_id")}
    sync = Node(package="mas_sync", executable="p2p_sync_node", output="screen", parameters=[common])
    task = Node(package="mas_task", executable="p2p_task_node", output="screen", parameters=[common])
    mission = Node(
        package="mas_mission", executable="mission_logic_node", output="screen",
        parameters=[{
            **common,
            "num_drones": ParameterValue(lc("num_drones"), value_type=int),
            "role_coordinator_id": lc("role_coordinator_id"),
            "arena_width": ParameterValue(lc("arena_width"), value_type=float),
            "arena_height": ParameterValue(lc("arena_height"), value_type=float),
            "require_flight_ready": True,
        }],
    )
    return LaunchDescription(arguments + [OpaqueFunction(function=_validate), slam, bridge, perception, sync, task, mission])
