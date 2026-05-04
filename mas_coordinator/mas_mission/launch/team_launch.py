"""
team_launch.py
--------------
Launches the full 4-drone MAS stack.

For each drone (d1..d4) it starts:
  - p2p_sync_node
  - p2p_task_node
  - mission_logic_node

Usage:
  ros2 launch mas_mission team_launch.py

Optional overrides:
  ros2 launch mas_mission team_launch.py mission_duration:=480.0 arena_width:=91.44
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DRONE_IDS = ["d1", "d2", "d3", "d4"]


def drone_group(drone_id: str,
                mission_duration, num_drones,
                arena_width, arena_height) -> list:
    """Return the three nodes for one drone (sync + task + mission)."""

    common_params = [
        {"drone_id":         drone_id},
        {"mission_duration": mission_duration},
        {"num_drones":       num_drones},
        {"arena_width":      arena_width},
        {"arena_height":     arena_height},
    ]

    sync_node = Node(
        package="mas_sync",
        executable="p2p_sync_node",
        name=f"p2p_sync_{drone_id}",
        namespace=drone_id,
        parameters=common_params + [
            {"r_enter":        8.0},
            {"r_exit":         12.0},
            {"t_sync":         5.0},
            {"hello_interval": 3.0},
            {"beacon_rate":    5.0},
        ],
        output="screen",
        remappings=[
            (f"/{drone_id}/team/pose_beacon", "/team/pose_beacon"),
            (f"/{drone_id}/team/sync_hello",  "/team/sync_hello"),
            (f"/{drone_id}/team/sync_ack",    "/team/sync_ack"),
            (f"/{drone_id}/team/mine_delta",  "/team/mine_delta"),
        ],
    )

    task_node = Node(
        package="mas_task",
        executable="p2p_task_node",
        name=f"p2p_task_{drone_id}",
        namespace=drone_id,
        parameters=common_params + [
            {"tick_rate":          5.0},
            {"announce_cooldown": 10.0},
        ],
        output="screen",
        remappings=[
            (f"/{drone_id}/team/task_announce", "/team/task_announce"),
            (f"/{drone_id}/team/task_claim",    "/team/task_claim"),
            (f"/{drone_id}/team/task_result",   "/team/task_result"),
        ],
    )

    mission_node = Node(
        package="mas_mission",
        executable="mission_logic_node",
        name=f"mission_logic_{drone_id}",
        namespace=drone_id,
        parameters=common_params,
        output="screen",
        remappings=[
            (f"/{drone_id}/team/pose_beacon",   "/team/pose_beacon"),
            (f"/{drone_id}/team/mine_delta",    "/team/mine_delta"),
            (f"/{drone_id}/team/task_announce", "/team/task_announce"),
            (f"/{drone_id}/team/task_result",   "/team/task_result"),
        ],
    )

    return [sync_node, task_node, mission_node]


def generate_launch_description():

    # ── Declare overridable arguments ─────────────────────────────────────────
    args = [
        DeclareLaunchArgument("mission_duration", default_value="420.0",
                              description="Total mission time in seconds (IARC: 420 s)"),
        DeclareLaunchArgument("num_drones",       default_value="4",
                              description="Number of drones in the team"),
        DeclareLaunchArgument("arena_width",      default_value="91.44",
                              description="Arena width in metres (IARC: 300 ft = 91.44 m)"),
        DeclareLaunchArgument("arena_height",     default_value="24.38",
                              description="Arena height in metres (IARC: 80 ft = 24.38 m)"),
    ]

    mission_duration = LaunchConfiguration("mission_duration")
    num_drones       = LaunchConfiguration("num_drones")
    arena_width      = LaunchConfiguration("arena_width")
    arena_height     = LaunchConfiguration("arena_height")

    # ── Build nodes for each drone ────────────────────────────────────────────
    all_nodes = []
    for drone_id in DRONE_IDS:
        all_nodes.extend(
            drone_group(drone_id,
                        mission_duration, num_drones,
                        arena_width, arena_height)
        )

    return LaunchDescription(args + all_nodes)
