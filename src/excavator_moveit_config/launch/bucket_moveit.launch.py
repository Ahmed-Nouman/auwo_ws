"""
MoveIt + RViz like demo_moveit_rviz, with an optional Gazebo digital-twin path.

  # Mock ros2_control only (no Gazebo): local /controller_manager + mock hardware
  ros2 launch excavator_moveit_config bucket_moveit.launch.py

  # Gazebo Harmonic + excavator in sim + Plan/Execute on the simulated arm
  ros2 launch excavator_moveit_config bucket_moveit.launch.py \\
    include_gazebo:=true use_sim_time:=true

  include_gazebo:=false starts the mock stack above; it does not attach to an
  already-running Gazebo. For auwo_twin + MoveIt, use a separate launch or
  domain that connects only to the existing sim (avoid two /controller_manager).

Defaults to ROS_DOMAIN_ID=42 like the demo.
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_moveit = get_package_share_directory("excavator_moveit_config")
    pkg_desc = get_package_share_directory("excavator_description")
    pkg_gazebo = get_package_share_directory("excavator_gazebo")
    controllers_yaml = os.path.join(pkg_desc, "config", "controllers.yaml")
    excavation_site_world = os.path.join(pkg_desc, "worlds", "excavation_site_local.sdf")

    use_sim = LaunchConfiguration("use_sim_time")
    include_gz = LaunchConfiguration("include_gazebo")

    # ---- Mock hardware (no Gazebo): same builder pattern as demo_moveit_rviz ----
    builder_mock = MoveItConfigsBuilder("excavator", package_name="excavator_moveit_config")
    builder_mock.planning_pipelines(pipelines=["ompl"])
    builder_mock.robot_description(mappings={"use_mock_hardware": "true"})
    moveit_mock = builder_mock.to_moveit_configs()
    robot_desc_mock = moveit_mock.robot_description

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_desc_mock,
            controllers_yaml,
            {"use_sim_time": ParameterValue(use_sim, value_type=bool)},
        ],
        condition=UnlessCondition(include_gz),
    )

    robot_state_publisher_mock = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_desc_mock,
            {"use_sim_time": ParameterValue(use_sim, value_type=bool)},
        ],
        condition=UnlessCondition(include_gz),
    )

    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=UnlessCondition(include_gz),
    )

    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=UnlessCondition(include_gz),
    )

    # ---- Gazebo (excavator in sim); ros2_control lives inside gz ----
    use_excavation_site = LaunchConfiguration("use_excavation_site")
    world_cfg = LaunchConfiguration("world")

    gazebo_excavation = GroupAction(
        condition=IfCondition(include_gz),
        actions=[
            GroupAction(
                condition=IfCondition(use_excavation_site),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(pkg_gazebo, "launch", "gazebo.launch.py"),
                        ),
                        launch_arguments={
                            "world": world_cfg,
                            "use_sim_time": use_sim,
                            "headless": LaunchConfiguration("headless"),
                            "gazebo_unified_gui": LaunchConfiguration(
                                "gazebo_unified_gui"
                            ),
                            "gazebo_verbose": LaunchConfiguration("gazebo_verbose"),
                            "controller_spawn_delay_sec": LaunchConfiguration(
                                "gazebo_controller_spawn_delay_sec"
                            ),
                            "spawn_x": "0.0",
                            "spawn_y": "0.0",
                            "spawn_z": "1.5",
                            "spawn_dumper": "true",
                            "dumper_x": "4.0",
                            "dumper_y": "3.0",
                            "dumper_z": "0.5",
                            "dumper_yaw": "0.0",
                        }.items(),
                    ),
                ],
            ),
        ],
    )

    gazebo_default = GroupAction(
        condition=IfCondition(include_gz),
        actions=[
            GroupAction(
                condition=UnlessCondition(use_excavation_site),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(pkg_gazebo, "launch", "gazebo.launch.py"),
                        ),
                        launch_arguments={
                            "world": world_cfg,
                            "use_sim_time": use_sim,
                            "headless": LaunchConfiguration("headless"),
                            "gazebo_unified_gui": LaunchConfiguration(
                                "gazebo_unified_gui"
                            ),
                            "gazebo_verbose": LaunchConfiguration("gazebo_verbose"),
                            "controller_spawn_delay_sec": LaunchConfiguration(
                                "gazebo_controller_spawn_delay_sec"
                            ),
                        }.items(),
                    ),
                ],
            ),
        ],
    )

    pub_truck = LaunchConfiguration("publish_truck_obstacle")
    truck_collision = GroupAction(
        condition=IfCondition(include_gz),
        actions=[
            GroupAction(
                condition=IfCondition(pub_truck),
                actions=[
                    Node(
                        package="excavator_moveit_config",
                        executable="publish_truck_collision_object.py",
                        name="truck_collision_object_publisher",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": ParameterValue(
                                    use_sim, value_type=bool
                                ),
                                "frame_id": "world",
                                "object_id": "dump_truck_box",
                                "position_x": ParameterValue(
                                    LaunchConfiguration("truck_box_x"),
                                    value_type=float,
                                ),
                                "position_y": ParameterValue(
                                    LaunchConfiguration("truck_box_y"),
                                    value_type=float,
                                ),
                                "position_z": ParameterValue(
                                    LaunchConfiguration("truck_box_z"),
                                    value_type=float,
                                ),
                                "yaw": ParameterValue(
                                    LaunchConfiguration("truck_box_yaw"),
                                    value_type=float,
                                ),
                                "box_length_x": ParameterValue(
                                    LaunchConfiguration("truck_box_lx"),
                                    value_type=float,
                                ),
                                "box_length_y": ParameterValue(
                                    LaunchConfiguration("truck_box_ly"),
                                    value_type=float,
                                ),
                                "box_length_z": ParameterValue(
                                    LaunchConfiguration("truck_box_lz"),
                                    value_type=float,
                                ),
                            }
                        ],
                    ),
                ],
            ),
        ],
    )

    move_group_common = {
        "use_sim_time": use_sim,
        "trajectory_action": "/arm_trajectory_controller/follow_joint_trajectory",
        "joint_states_topic": "/joint_states",
        "body_rotation_planning_min": LaunchConfiguration("body_rotation_planning_min"),
        "body_rotation_planning_max": LaunchConfiguration("body_rotation_planning_max"),
    }

    move_group_mock_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "move_group.launch.py")
        ),
        launch_arguments={**move_group_common, "use_mock_hardware": "true"}.items(),
    )

    move_group_sim_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "move_group.launch.py")
        ),
        launch_arguments={**move_group_common, "use_mock_hardware": "false"}.items(),
    )

    moveit_rviz_mock_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "moveit_rviz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim,
            "use_mock_hardware": "true",
            "joint_states_topic": "/joint_states",
        }.items(),
    )

    moveit_rviz_sim_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "moveit_rviz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim,
            "use_mock_hardware": "false",
            "joint_states_topic": "/joint_states",
        }.items(),
    )

    wait_script = os.path.join(
        get_package_prefix("excavator_moveit_config"),
        "lib",
        "excavator_moveit_config",
        "wait_for_arm_trajectory_action.py",
    )

    wait_mock = ExecuteProcess(
        cmd=[
            "python3",
            wait_script,
            "--timeout",
            LaunchConfiguration("wait_move_group_timeout_sec"),
            "--controller-manager",
            "/controller_manager",
            "--action",
            "/arm_trajectory_controller/follow_joint_trajectory",
        ],
        output="screen",
    )

    wait_gazebo = ExecuteProcess(
        cmd=[
            "python3",
            wait_script,
            "--timeout",
            LaunchConfiguration("wait_move_group_timeout_sec"),
            "--controller-manager",
            "/controller_manager",
            "--action",
            "/arm_trajectory_controller/follow_joint_trajectory",
        ],
        output="screen",
    )

    def _on_wait_mock_exit(event, context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        "[bucket_moveit] wait_for_arm_trajectory_action failed (exit %d)."
                        % event.returncode
                    )
                ),
            ]
        actions = [move_group_mock_ld]
        try:
            rviz_on = context.perform_substitution(LaunchConfiguration("launch_rviz"))
        except Exception:
            rviz_on = "true"
        if str(rviz_on).lower() in ("true", "1"):
            actions.append(moveit_rviz_mock_ld)
        return actions

    def _on_wait_gazebo_exit(event, context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        "[bucket_moveit] wait_for_arm_trajectory_action failed (exit %d)."
                        % event.returncode
                    )
                ),
            ]
        actions = [move_group_sim_ld]
        try:
            rviz_on = context.perform_substitution(LaunchConfiguration("launch_rviz"))
        except Exception:
            rviz_on = "true"
        if str(rviz_on).lower() in ("true", "1"):
            actions.append(moveit_rviz_sim_ld)
        return actions

    when_wait_mock_done = RegisterEventHandler(
        OnProcessExit(target_action=wait_mock, on_exit=_on_wait_mock_exit),
        condition=UnlessCondition(include_gz),
    )

    after_jsb_spawn_arm = RegisterEventHandler(
        OnProcessExit(target_action=spawner_jsb, on_exit=[spawner_arm]),
        condition=UnlessCondition(include_gz),
    )

    after_arm_spawn_wait_mock = RegisterEventHandler(
        OnProcessExit(target_action=spawner_arm, on_exit=[wait_mock]),
        condition=UnlessCondition(include_gz),
    )

    spawn_controllers_mock = TimerAction(
        period=3.0,
        actions=[spawner_jsb, after_jsb_spawn_arm, after_arm_spawn_wait_mock],
        condition=UnlessCondition(include_gz),
    )

    when_wait_gazebo_done = RegisterEventHandler(
        OnProcessExit(target_action=wait_gazebo, on_exit=_on_wait_gazebo_exit),
        condition=IfCondition(include_gz),
    )

    delayed_wait_gazebo = TimerAction(
        period=LaunchConfiguration("gazebo_controller_ready_delay_sec"),
        actions=[wait_gazebo],
        condition=IfCondition(include_gz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value="42",
                description="ROS domain for this stack.",
            ),
            SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
            DeclareLaunchArgument(
                "include_gazebo",
                default_value="false",
                description=(
                    "If true: start excavator_gazebo (gz + controllers in sim). "
                    "If false: local ros2_control mock only (not external Gazebo attach)."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Set true when using Gazebo /clock (include_gazebo:=true).",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Start MoveIt RViz after the arm trajectory wait succeeds.",
            ),
            DeclareLaunchArgument(
                "wait_move_group_timeout_sec",
                default_value="120.0",
                description="Seconds to wait for active controllers + FollowJointTrajectory action.",
            ),
            DeclareLaunchArgument(
                "use_excavation_site",
                default_value="true",
                description="When include_gazebo: excavation world + dump truck spawn.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=excavation_site_world,
                description="World SDF when include_gazebo true",
            ),
            DeclareLaunchArgument(
                "publish_truck_obstacle",
                default_value="true",
                description="Publish coarse truck box on /collision_object (Gazebo path).",
            ),
            DeclareLaunchArgument(
                "gazebo_controller_ready_delay_sec",
                default_value="35.0",
                description=(
                    "include_gazebo: seconds before wait script runs (after gz spawn + spawners)."
                ),
            ),
            DeclareLaunchArgument(
                "gazebo_controller_spawn_delay_sec",
                default_value="12.0",
                description="Forwarded to excavator_gazebo before loading controllers.",
            ),
            DeclareLaunchArgument(
                "body_rotation_planning_min",
                default_value="-3.141592653589793",
                description="Planning-only lower bound for body_rotation (rad)",
            ),
            DeclareLaunchArgument(
                "body_rotation_planning_max",
                default_value="3.141592653589793",
                description="Planning-only upper bound for body_rotation (rad)",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Forwarded to excavator_gazebo.",
            ),
            DeclareLaunchArgument(
                "gazebo_unified_gui",
                default_value="false",
                description="Forwarded to excavator_gazebo.",
            ),
            DeclareLaunchArgument(
                "gazebo_verbose",
                default_value="1",
                description="Forwarded to excavator_gazebo.",
            ),
            DeclareLaunchArgument("truck_box_x", default_value="6.25"),
            DeclareLaunchArgument("truck_box_y", default_value="3.0"),
            DeclareLaunchArgument("truck_box_z", default_value="0.95"),
            DeclareLaunchArgument("truck_box_yaw", default_value="0.0"),
            DeclareLaunchArgument("truck_box_lx", default_value="4.5"),
            DeclareLaunchArgument("truck_box_ly", default_value="2.6"),
            DeclareLaunchArgument("truck_box_lz", default_value="1.7"),
            # Mock path (demo-style)
            when_wait_mock_done,
            ros2_control_node,
            robot_state_publisher_mock,
            spawn_controllers_mock,
            # Gazebo path (move_group + RViz only after wait_for_arm_trajectory_action succeeds)
            when_wait_gazebo_done,
            gazebo_excavation,
            gazebo_default,
            delayed_wait_gazebo,
            truck_collision,
        ]
    )
