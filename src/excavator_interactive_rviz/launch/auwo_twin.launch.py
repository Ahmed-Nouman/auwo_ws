import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_desc = get_package_share_directory("excavator_description")
    pkg_gazebo = get_package_share_directory("excavator_gazebo")
    empty_world = os.path.join(pkg_desc, "worlds", "empty.sdf")
    excavation_site_world = os.path.join(pkg_desc, "worlds", "excavation_site_local.sdf")

    use_excavation_site = LaunchConfiguration("use_excavation_site")
    world_cfg = LaunchConfiguration("world")

    # --- Gazebo: excavation site (world + dump truck) ---
    gazebo_excavation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_gazebo, "/launch/gazebo.launch.py"],
        ),
        launch_arguments={
            "world": world_cfg,
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

    # --- Gazebo: default empty world (no truck) ---
    gazebo_default = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_gazebo, "/launch/gazebo.launch.py"],
        ),
        launch_arguments={"world": world_cfg}.items(),
        condition=UnlessCondition(use_excavation_site),
    )

    # --- RViz ---
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("excavator_description"), "config", "view.rviz"]
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="auwo_digital_twin_rviz",
        arguments=["-d", rviz_config],
        output="screen",
    )
    rviz_delayed = TimerAction(period=4.0, actions=[rviz])

    # --- Interactive markers ---
    joint_imarkers = Node(
        package="excavator_interactive_rviz",
        executable="joint_imarkers.py",
        name="excavator_joint_imarkers",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # --- Twin router ---
    twin_router_node = Node(
        package="excavator_interactive_rviz",
        executable="twin_router_node.py",
        name="excavator_twin_router",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"default_mode": "simulation"},
            {"sim_command_topic": "/arm_position_controller/commands"},
            {"sim_state_topic": "/joint_states"},
            {"physical_command_topic": "/physical_twin/commands"},
            {"physical_state_topic": "/physical_twin/state"},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_excavation_site",
                default_value="true",
                description="Use excavation_site world and spawn dump truck",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=excavation_site_world,
                description="Path to world SDF. Default: excavation_site_local.sdf (terrain from pete-driving). Use empty.sdf for plain ground.",
            ),
            gazebo_excavation,
            gazebo_default,
            rviz_delayed,
            joint_imarkers,
            twin_router_node,
        ]
    )
