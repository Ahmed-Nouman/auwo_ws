"""
MoveIt + RViz only: mock ros2_control (no Gazebo).

Use this to verify Plan and Execute against a local FollowJointTrajectory server
and consistent /joint_states.

  ros2 launch excavator_moveit_config demo_moveit_rviz.launch.py

This launch defaults to an isolated ROS_DOMAIN_ID to avoid cross-talk with
other running stacks.
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_moveit = get_package_share_directory("excavator_moveit_config")
    pkg_desc = get_package_share_directory("excavator_description")
    controllers_yaml = os.path.join(pkg_desc, "config", "controllers.yaml")

    use_sim = LaunchConfiguration("use_sim_time")

    builder = MoveItConfigsBuilder("excavator", package_name="excavator_moveit_config")
    builder.planning_pipelines(pipelines=["ompl"])
    builder.robot_description(mappings={"use_mock_hardware": "true"})
    moveit_config = builder.to_moveit_configs()
    robot_desc_dict = moveit_config.robot_description

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_desc_dict,
            controllers_yaml,
            {"use_sim_time": ParameterValue(use_sim, value_type=bool)},
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_desc_dict,
            {"use_sim_time": ParameterValue(use_sim, value_type=bool)},
        ],
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
    )

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
            "--controller-manager",
            "/controller_manager",
            "--action",
            "/arm_trajectory_controller/follow_joint_trajectory",
        ],
        output="screen",
    )

    move_group_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "move_group.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim,
            "use_mock_hardware": "true",
            "trajectory_action": "/arm_trajectory_controller/follow_joint_trajectory",
            "joint_states_topic": "/joint_states",
        }.items(),
    )

    moveit_rviz_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "moveit_rviz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim,
            "use_mock_hardware": "true",
            "joint_states_topic": "/joint_states",
        }.items(),
    )

    def _on_wait_exit(event, context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        "[demo_moveit_rviz] wait_for_arm_trajectory_action failed (exit %d)."
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

    when_wait_done = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_trajectory_action,
            on_exit=_on_wait_exit,
        )
    )

    after_jsb_spawn_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=spawner_jsb,
            on_exit=[spawner_arm],
        )
    )

    after_arm_spawn_wait = RegisterEventHandler(
        OnProcessExit(
            target_action=spawner_arm,
            on_exit=[wait_trajectory_action],
        )
    )

    spawn_controllers = TimerAction(
        period=3.0,
        actions=[spawner_jsb, after_jsb_spawn_arm, after_arm_spawn_wait],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value="42",
                description="Isolated ROS domain for mock MoveIt demo.",
            ),
            SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Wall clock for mock hardware (set true only if driving /clock).",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Start RViz with MotionPlanning after controllers are ready.",
            ),
            DeclareLaunchArgument(
                "wait_move_group_timeout_sec",
                default_value="120.0",
                description="Seconds to wait for arm trajectory action + active controllers.",
            ),
            when_wait_done,
            ros2_control_node,
            robot_state_publisher,
            spawn_controllers,
        ]
    )
