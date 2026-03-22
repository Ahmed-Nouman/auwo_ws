import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
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

    # Force Gazebo to paused state shortly after startup.
    # This keeps auwo_twin behavior deterministic even if gz sim starts running.
    pause_gazebo = ExecuteProcess(
        cmd=[
            "gz",
            "service",
            "-s",
            "/world/default/control",
            "--reqtype",
            "gz.msgs.WorldControl",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            "pause: true",
        ],
        output="screen",
    )
    pause_gazebo_delayed = TimerAction(period=5.0, actions=[pause_gazebo])

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

    # --- Trajectory adapter: position commands -> smooth JointTrajectory for arm_trajectory_controller ---
    trajectory_adapter = Node(
        package="excavator_teleop",
        executable="trajectory_command_adapter.py",
        name="trajectory_command_adapter",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # --- IMU -> Pose for RViz (orientation from /imu at fixed position) ---
    imu_to_pose = Node(
        package="excavator_gazebo",
        executable="imu_to_pose.py",
        name="imu_to_pose",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"pose_frame_id": "world"},
            {"position_x": 0.5},
            {"position_y": 0.0},
            {"position_z": 1.5},
        ],
    )

    # --- Point cloud frame remap: /points (Gazebo frame) -> /points_viz (sensor_lidar_link) for RViz ---
    points_frame_remap = Node(
        package="excavator_gazebo",
        executable="points_frame_remap.py",
        name="points_frame_remap",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"target_frame_id": "sensor_lidar_link"},
            {"input_topic": "/points"},
            {"output_topic": "/points_viz"},
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
            pause_gazebo_delayed,
            rviz_delayed,
            joint_imarkers,
            twin_router_node,
            trajectory_adapter,
            imu_to_pose,
            points_frame_remap,
        ]
    )
