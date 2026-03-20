"""
All-in-one MoveIt2 + Gazebo + MoveIt RViz for the excavator arm.

  ros2 launch excavator_moveit_config bucket_moveit.launch.py

Gazebo already running (e.g. auwo_twin) — no second Gazebo/RViz:

  ros2 launch excavator_moveit_config bucket_moveit.launch.py \\
    include_gazebo:=false launch_rviz:=false
"""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_prefix, get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("excavator_moveit_config")
    pkg_desc = get_package_share_directory("excavator_description")
    pkg_gazebo = get_package_share_directory("excavator_gazebo")
    excavation_site_world = os.path.join(pkg_desc, "worlds", "excavation_site_local.sdf")

    use_sim = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    pub_truck = LaunchConfiguration("publish_truck_obstacle")
    include_gz = LaunchConfiguration("include_gazebo")
    use_excavation_site = LaunchConfiguration("use_excavation_site")
    world_cfg = LaunchConfiguration("world")

    gazebo_excavation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "gazebo.launch.py"),
        ),
        launch_arguments={
            "world": world_cfg,
            "use_sim_time": use_sim,
            "headless": LaunchConfiguration("headless"),
            "gazebo_unified_gui": LaunchConfiguration("gazebo_unified_gui"),
            "gazebo_verbose": LaunchConfiguration("gazebo_verbose"),
            "spawn_x": "0.0",
            "spawn_y": "0.0",
            "spawn_z": "1.5",
            "spawn_dumper": "true",
            "dumper_x": "4.0",
            "dumper_y": "3.0",
            "dumper_z": "0.5",
            "dumper_yaw": "0.0",
        }.items(),
        condition=IfCondition(use_excavation_site),
    )

    gazebo_default = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "gazebo.launch.py"),
        ),
        launch_arguments={
            "world": world_cfg,
            "use_sim_time": use_sim,
            "headless": LaunchConfiguration("headless"),
            "gazebo_unified_gui": LaunchConfiguration("gazebo_unified_gui"),
            "gazebo_verbose": LaunchConfiguration("gazebo_verbose"),
        }.items(),
        condition=UnlessCondition(use_excavation_site),
    )

    gazebo_stack = GroupAction(
        actions=[gazebo_excavation, gazebo_default],
        condition=IfCondition(include_gz),
    )

    truck_collision = Node(
        package="excavator_moveit_config",
        executable="publish_truck_collision_object.py",
        name="truck_collision_object_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim, value_type=bool),
                "frame_id": "world",
                "object_id": "dump_truck_box",
                "position_x": ParameterValue(
                    LaunchConfiguration("truck_box_x"), value_type=float
                ),
                "position_y": ParameterValue(
                    LaunchConfiguration("truck_box_y"), value_type=float
                ),
                "position_z": ParameterValue(
                    LaunchConfiguration("truck_box_z"), value_type=float
                ),
                "yaw": ParameterValue(
                    LaunchConfiguration("truck_box_yaw"), value_type=float
                ),
                "box_length_x": ParameterValue(
                    LaunchConfiguration("truck_box_lx"), value_type=float
                ),
                "box_length_y": ParameterValue(
                    LaunchConfiguration("truck_box_ly"), value_type=float
                ),
                "box_length_z": ParameterValue(
                    LaunchConfiguration("truck_box_lz"), value_type=float
                ),
            }
        ],
        condition=IfCondition(pub_truck),
    )

    move_group_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "move_group.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim}.items(),
    )

    moveit_rviz_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "moveit_rviz.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim}.items(),
    )

    # Start move_group only after /arm_trajectory_controller/follow_joint_trajectory exists
    # (avoids fixed delays and "Action client not connected" on fast/slow machines).
    wait_script = os.path.join(
        get_package_prefix("excavator_moveit_config"),
        "lib",
        "excavator_moveit_config",
        "wait_for_arm_trajectory_action.py",
    )
    wait_trajectory_action = ExecuteProcess(
        cmd=[
            "python3",
            wait_script,
            "--timeout",
            LaunchConfiguration("wait_move_group_timeout_sec"),
        ],
        output="screen",
    )

    def _on_wait_trajectory_exit(event, context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        "[bucket_moveit] wait_for_arm_trajectory_action failed (exit %d); "
                        "skipping move_group. Fix Gazebo spawn / ros2_control, then relaunch."
                        % event.returncode
                    )
                ),
            ]
        actions = [move_group_ld]
        try:
            rviz_on = context.perform_substitution(LaunchConfiguration("launch_rviz"))
        except Exception:
            rviz_on = "true"
        if str(rviz_on).lower() in ("true", "1"):
            actions.append(moveit_rviz_ld)
        return actions

    move_group_when_gazebo = GroupAction(
        actions=[
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_trajectory_action,
                    on_exit=_on_wait_trajectory_exit,
                )
            ),
            wait_trajectory_action,
        ],
        condition=IfCondition(include_gz),
    )
    move_group_when_no_gazebo = GroupAction(
        actions=[move_group_ld],
        condition=UnlessCondition(include_gz),
    )

    # When include_gazebo:=false, start RViz on a 4s timer (move_group starts immediately).
    # When include_gazebo:=true, RViz is launched from _on_wait_trajectory_exit after controllers are ready.
    rviz_delayed_twin_only = GroupAction(
        condition=UnlessCondition(include_gz),
        actions=[
            GroupAction(
                condition=IfCondition(launch_rviz),
                actions=[
                    TimerAction(
                        period=4.0,
                        actions=[moveit_rviz_ld],
                    ),
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Gazebo /clock and MoveIt nodes",
            ),
            DeclareLaunchArgument(
                "include_gazebo",
                default_value="true",
                description="If false, only move_group (+ RViz); start Gazebo elsewhere.",
            ),
            DeclareLaunchArgument(
                "use_excavation_site",
                default_value="true",
                description="When include_gazebo true: excavation world + dump truck spawn (matches auwo_twin).",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=excavation_site_world,
                description="World SDF when include_gazebo true",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Start RViz with config/moveit.rviz (MotionPlanning + move_group)",
            ),
            DeclareLaunchArgument(
                "publish_truck_obstacle",
                default_value="true",
                description="Publish coarse truck box on /collision_object",
            ),
            DeclareLaunchArgument(
                "wait_move_group_timeout_sec",
                default_value="180.0",
                description=(
                    "When include_gazebo:=true, max seconds to wait for "
                    "/arm_trajectory_controller/follow_joint_trajectory before giving up"
                ),
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description=(
                    "Forwarded to excavator_gazebo: if true, run Gazebo server only (no gz sim -g GUI)."
                ),
            ),
            DeclareLaunchArgument(
                "gazebo_unified_gui",
                default_value="false",
                description=(
                    "Forwarded to excavator_gazebo: single gz sim (GUI+server) when true; "
                    "split -s/-g when false (default)."
                ),
            ),
            DeclareLaunchArgument(
                "gazebo_verbose",
                default_value="1",
                description="Forwarded to excavator_gazebo: gz sim -v level (0–4).",
            ),
            DeclareLaunchArgument("truck_box_x", default_value="4.0"),
            DeclareLaunchArgument("truck_box_y", default_value="3.0"),
            DeclareLaunchArgument("truck_box_z", default_value="1.15"),
            DeclareLaunchArgument("truck_box_yaw", default_value="0.0"),
            DeclareLaunchArgument("truck_box_lx", default_value="7.0"),
            DeclareLaunchArgument("truck_box_ly", default_value="2.8"),
            DeclareLaunchArgument("truck_box_lz", default_value="2.4"),
            gazebo_stack,
            move_group_when_gazebo,
            move_group_when_no_gazebo,
            truck_collision,
            rviz_delayed_twin_only,
        ]
    )
