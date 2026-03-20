"""RViz2 + MotionPlanning plugin (AUWO excavator)."""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, add_debuggable_node


def generate_launch_description():
    builder = MoveItConfigsBuilder("excavator", package_name="excavator_moveit_config")
    builder.planning_pipelines(pipelines=["ompl"], load_all=False)
    moveit_config = builder.to_moveit_configs()

    # Same as move_group: thin overlays sometimes miss OMPL plugin paths for RViz/MotionPlanning.
    extra_env = {"DISPLAY": os.environ.get("DISPLAY", "")}
    try:
        ompl_prefix = get_package_prefix("moveit_planners_ompl")
        ament = os.environ.get("AMENT_PREFIX_PATH", "")
        extra_env["AMENT_PREFIX_PATH"] = (
            f"{ompl_prefix}:{ament}" if ament else ompl_prefix
        )
    except PackageNotFoundError:
        pass

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo /clock",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    # RViz MotionPlanning needs the same URDF/SRDF as move_group (not only kinematics).
    rviz_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            )
        },
    ]

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
        additional_env=extra_env,
    )

    return ld
