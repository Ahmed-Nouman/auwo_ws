import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Package directories ──────────────────────────────────────────────────
    pkg_desc   = get_package_share_directory("excavator_description")
    pkg_gazebo = get_package_share_directory("excavator_gazebo")
    pkg_twin   = get_package_share_directory("excavator_interactive_rviz")

    # ── File paths ───────────────────────────────────────────────────────────
    default_model        = os.path.join(pkg_desc, "urdf", "excavator.urdf.xacro")
    default_world        = os.path.join(pkg_desc, "worlds", "empty.sdf")
    follower_params_file = os.path.join(pkg_twin, "config", "physical_tf_follower.yaml")

    # ── Launch configurations ────────────────────────────────────────────────
    use_gazebo    = LaunchConfiguration("use_gazebo")
    spawn_dumper  = LaunchConfiguration("spawn_dumper")
    world         = LaunchConfiguration("world")
    model         = LaunchConfiguration("model")
    use_sim_time  = LaunchConfiguration("use_sim_time")
    use_demo_pose = LaunchConfiguration("use_demo_pose")
    use_novatron  = LaunchConfiguration("use_novatron")

    # MQTT connection parameters as CLI-overridable arguments so the launch
    # file works even without a novatron_params.yaml file on disk.
    # Override at runtime:
    #   ros2 launch ... mqtt_host:=192.168.4.232 mqtt_port:=8884
    mqtt_host     = LaunchConfiguration("mqtt_host")
    mqtt_port     = LaunchConfiguration("mqtt_port")
    mqtt_topic    = LaunchConfiguration("mqtt_topic")
    mqtt_username = LaunchConfiguration("mqtt_username")
    mqtt_password = LaunchConfiguration("mqtt_password")

    # ── Robot description ────────────────────────────────────────────────────
    # use_sim:= must match whether Gazebo is actually running.
    # Passing use_sim:=true without Gazebo causes a fatal controller_manager
    # crash because GazeboSimSystem can't find the simulation.
    robot_description = ParameterValue(
        Command([
            TextSubstitution(text="xacro "),
            model,
            TextSubstitution(text=" use_sim:="),
            PythonExpression([
                "'true' if '", use_gazebo, "'.lower() in ('true','1') else 'false'"
            ]),
        ]),
        value_type=str,
    )

    # ────────────────────────────────────────────────────────────────────────
    # CORE NODES — always active
    # ────────────────────────────────────────────────────────────────────────

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )

    # FIX: robot_description_topics.py crashes with "Executor is already
    # spinning" when it tries to call rclpy.spin_until_future_complete()
    # from inside its own timer callback (which is already inside rclpy.spin).
    # The node accepts fallback_excavator_description for exactly this case.
    # We delay it by 2 s so robot_state_publisher is fully initialised before
    # the node's timer fires and attempts the async parameter fetch.
    # If the upstream bug is fixed in a future package version, the TimerAction
    # wrapper can be removed without any other changes.
    robot_description_topics = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="excavator_interactive_rviz",
                executable="robot_description_topics.py",
                name="robot_description_topics",
                output="screen",
                parameters=[{
                    "fallback_excavator_description": robot_description,
                }],
            )
        ],
    )

    twin_router_node = Node(
        package="excavator_interactive_rviz",
        executable="twin_router_node.py",
        name="excavator_twin_router",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"default_mode": "physical"},
            {"sim_command_topic": "/arm_position_controller/commands"},
            {"sim_state_topic": "/joint_states"},
            {"physical_command_topic": "/physical_twin/commands"},
            {"physical_state_topic": "/physical_twin/state"},
        ],
    )

    # ────────────────────────────────────────────────────────────────────────
    # PHYSICAL TF FOLLOWER
    # ────────────────────────────────────────────────────────────────────────
    physical_tf_follower = Node(
        package="excavator_interactive_rviz",
        executable="physical_tf_follower_node.py",
        name="physical_tf_follower_node",
        output="screen",
        parameters=[
            follower_params_file,
            {
                # Enable joint mirroring only in live physical mode — not in
                # demo mode (which provides its own joint states) and not in
                # Gazebo mode (which has ros2_control doing that job).
                "mirror_to_joint_states": PythonExpression([
                    "'", use_demo_pose, "'.lower() not in ('true','1')",
                    " and '", use_gazebo, "'.lower() not in ('true','1')",
                ]),
                # Publish world→base_link only in live physical mode.
                # Gazebo uses imu_to_pose; demo uses demo_world_tf.
                "publish_world_to_base_tf": PythonExpression([
                    "'", use_gazebo,    "'.lower() not in ('true','1')",
                    " and '", use_demo_pose, "'.lower() not in ('true','1')",
                ]),
            },
        ],
    )

    # ────────────────────────────────────────────────────────────────────────
    # NOVATRON KINEMATIC PUBLISHER
    #
    # FIX: novatron_params.yaml did not exist, causing mqtt_port to fall back
    # to the node default of 0, which paho rejects with "Invalid port number".
    # MQTT parameters are now passed directly as node parameters in this launch
    # file, so no external yaml file is needed. Override any of them on the
    # command line:
    #   ros2 launch ... mqtt_host:=192.168.4.232 mqtt_port:=8884
    #
    # sensor_id_to_frame maps documented Novatron IDs (from package README)
    # to descriptive frame names. "novatron_" prefix keeps them visually
    # distinct from URDF links in RViz's TF display.
    # Undocumented IDs (1011, 1099, 2106, 2511, 3004, 3006, 3101, 3105)
    # fall back to kinematic_XXXX automatically — no entry needed for them.
    # ────────────────────────────────────────────────────────────────────────
    novatron_node = Node(
        package="novatron_xsite3d_interface",
        executable="kinematic_results_publisher_node",
        name="kinematic_results_publisher_node",
        output="screen",
        parameters=[{
            "mqtt_host":     mqtt_host,
            "mqtt_port":     mqtt_port,
            "mqtt_topic":    mqtt_topic,
            "mqtt_username": mqtt_username,
            "mqtt_password": mqtt_password,
            "sensor_id_to_frame": [
                # ── Body / base ──────────────────────────────────────────
                "1000:novatron_body_yaw",    # rotation axis, yaw only
                "1005:novatron_body_6dof",   # full 6DOF — primary body pose

                # ── Arm chain ────────────────────────────────────────────
                "1003:novatron_boom_joint",  # boom_rotation in URDF
                "1002:novatron_stick_joint", # stick_rotation in URDF
                "1001:novatron_bucket_joint",# bucket_rotation in URDF

                # ── Attachment (tilt-rotator / quick coupler) ────────────
                "2001:novatron_quick_coupler",
                "2002:novatron_tilt_base",
                "2003:novatron_tilt_joint",
                "2004:novatron_rotator_joint",

                # ── Bucket blade measurement points ──────────────────────
                "3001:novatron_blade_left",
                "3002:novatron_blade_center",
                "3003:novatron_blade_right",
                "3005:novatron_blade_bottom",
            ],
        }],
        condition=IfCondition(use_novatron),
    )

    # ────────────────────────────────────────────────────────────────────────
    # DEMO MODE NODES  (use_demo_pose:=true only)
    # ────────────────────────────────────────────────────────────────────────
    demo_joint_states = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="demo_joint_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"rate": 30},
            {"robot_description": robot_description},
            {"body_rotation":   0.0},
            {"boom_rotation":  -0.70},
            {"stick_rotation": -1.26},
            {"bucket_rotation":-1.12},
        ],
        condition=IfCondition(use_demo_pose),
    )

    demo_world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="demo_world_to_base_tf",
        arguments=[
            "--x",     "0.0",
            "--y",     "0.0",
            "--z",     "0.0",
            "--roll",  "0.0",
            "--pitch", "0.0",
            "--yaw",   "0.0",
            "--frame-id",       "world",
            "--child-frame-id", "base_link",
        ],
        condition=IfCondition(use_demo_pose),
    )

    # ────────────────────────────────────────────────────────────────────────
    # RViz — delayed 5 s so RSP and Novatron TF frames are ready first
    # (bumped from 4 s to account for the 2 s robot_description_topics delay)
    # ────────────────────────────────────────────────────────────────────────
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("excavator_description"), "config", "view.rviz"]
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="auwo_physical_twin_rviz",
        arguments=["-d", rviz_config],
        output="screen",
    )
    rviz_delayed = TimerAction(period=5.0, actions=[rviz])

    # ────────────────────────────────────────────────────────────────────────
    # GAZEBO-ONLY NODES  (use_gazebo:=true only)
    # ────────────────────────────────────────────────────────────────────────
    pause_gazebo = ExecuteProcess(
        cmd=[
            "gz", "service",
            "-s", "/world/default/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "5000",
            "--req", "pause: true",
        ],
        output="screen",
        condition=IfCondition(use_gazebo),
    )
    pause_gazebo_delayed = TimerAction(period=5.0, actions=[pause_gazebo])

    joint_imarkers = Node(
        package="excavator_interactive_rviz",
        executable="joint_imarkers.py",
        name="excavator_joint_imarkers",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_gazebo),
    )

    imu_to_pose = Node(
        package="excavator_gazebo",
        executable="imu_to_pose.py",
        name="imu_to_pose",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"pose_frame_id": "world"},
            {"position_x": 0.5},
            {"position_y": 0.0},
            {"position_z": 1.5},
        ],
        condition=IfCondition(use_gazebo),
    )

    points_frame_remap = Node(
        package="excavator_gazebo",
        executable="points_frame_remap.py",
        name="points_frame_remap",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"target_frame_id": "sensor_lidar_link"},
            {"input_topic": "/points"},
            {"output_topic": "/points_viz"},
        ],
        condition=IfCondition(use_gazebo),
    )

    trajectory_adapter = Node(
        package="excavator_teleop",
        executable="trajectory_command_adapter.py",
        name="trajectory_command_adapter",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_gazebo),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_gazebo, "/launch/gazebo.launch.py"]),
        launch_arguments={
            "world":        world,
            "spawn_dumper": spawn_dumper,
            "spawn_x":      "0.0",
            "spawn_y":      "0.0",
            "spawn_z":      "1.5",
            "dumper_x":     "4.0",
            "dumper_y":     "3.0",
            "dumper_z":     "0.5",
            "dumper_yaw":   "0.0",
        }.items(),
        condition=IfCondition(use_gazebo),
    )

    # ────────────────────────────────────────────────────────────────────────
    # ASSEMBLY
    # ────────────────────────────────────────────────────────────────────────
    return LaunchDescription([

        # ── Arguments ────────────────────────────────────────────────────────
        DeclareLaunchArgument("use_gazebo",    default_value="false",
            description="Launch Gazebo and spawn excavator."),
        DeclareLaunchArgument("spawn_dumper",  default_value="false",
            description="Spawn dump truck (requires use_gazebo:=true)."),
        DeclareLaunchArgument("world",         default_value=default_world,
            description="Gazebo world SDF path."),
        DeclareLaunchArgument("model",         default_value=default_model,
            description="Excavator URDF/Xacro path."),
        DeclareLaunchArgument("use_sim_time",
            default_value=PythonExpression(["'", use_gazebo, "'.lower() in ('true','1')"]),
            description="Use Gazebo /clock. Auto-derived from use_gazebo."),

        # FIX: was 'true' — caused static TF to override live Novatron data.
        DeclareLaunchArgument("use_demo_pose", default_value="false",
            description="true=static demo pose  false=live Novatron (default)."),

        DeclareLaunchArgument("use_novatron",  default_value="true",
            description="true=start MQTT publisher  false=skip (offline/Gazebo only)."),

        # MQTT parameters — set these to match your Xsite3D network config.
        # Defaults match the values in the novatron_xsite3d_interface README.
        DeclareLaunchArgument("mqtt_host",     default_value="192.168.4.232",
            description="IP address of the Xsite3D MQTT broker."),
        DeclareLaunchArgument("mqtt_port",     default_value="8884",
            description="Port of the Xsite3D MQTT broker."),
        DeclareLaunchArgument("mqtt_topic",
            default_value="novatron/realtime-app/kinematicResults",
            description="MQTT topic for kinematic results."),
        DeclareLaunchArgument("mqtt_username", default_value="user",
            description="MQTT broker username."),
        DeclareLaunchArgument("mqtt_password", default_value="password",
            description="MQTT broker password."),

        # ── Core ─────────────────────────────────────────────────────────────
        rsp,
        robot_description_topics,   # delayed 2 s — see comment on node def
        twin_router_node,
        physical_tf_follower,

        # ── Novatron ─────────────────────────────────────────────────────────
        novatron_node,

        # ── Demo mode ────────────────────────────────────────────────────────
        demo_joint_states,
        demo_world_tf,

        # ── RViz ─────────────────────────────────────────────────────────────
        rviz_delayed,

        # ── Gazebo ───────────────────────────────────────────────────────────
        gazebo,
        pause_gazebo_delayed,
        joint_imarkers,
        trajectory_adapter,
        imu_to_pose,
        points_frame_remap,
    ])