#!/usr/bin/env python3
# Gazebo Harmonic launcher for auwo_excavator_description (ROS 2 Jazzy)
import os, shutil
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    EnvironmentVariable,
    TextSubstitution,
    Command,
    PathJoinSubstitution,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

MINIMAL_WORLD_SDF = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="auwo_empty">
    <gravity>0 0 -9.81</gravity>
    <physics name="ode" type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1.0</ambient>
      <background>0.7 0.7 0.7 1.0</background>
    </scene>
    <include>
      <uri>model://ground_plane</uri>
    </include>
  </world>
</sdf>
"""

def _ensure_local_world(path: str) -> str:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        with open(path, "w") as f:
            f.write(MINIMAL_WORLD_SDF)
    return path

def _register_local_model(pkg_share: str):
    """Create ~/.gz/models/auwo_excavator_description so model:// URIs can resolve locally."""
    home = os.path.expanduser("~")
    models_root = os.path.join(home, ".gz", "models")
    model_root = os.path.join(models_root, "auwo_excavator_description")
    meshes_src = os.path.join(pkg_share, "meshes")
    meshes_dst = os.path.join(model_root, "meshes")

    os.makedirs(models_root, exist_ok=True)
    os.makedirs(model_root, exist_ok=True)

    config_path = os.path.join(model_root, "model.config")
    if not os.path.exists(config_path):
        with open(config_path, "w") as f:
            f.write("""<?xml version="1.0"?>
<model>
  <name>auwo_excavator_description</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>local</name><email>none@example.com</email></author>
  <description>Local shim model for ROS package meshes</description>
</model>
""")

    sdf_path = os.path.join(model_root, "model.sdf")
    if not os.path.exists(sdf_path):
        with open(sdf_path, "w") as f:
            f.write("""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="auwo_excavator_description">
    <static>true</static>
    <link name="dummy"/>
  </model>
</sdf>
""")

    # Ensure meshes symlink points to package meshes
    if os.path.lexists(meshes_dst):
        if os.path.islink(meshes_dst):
            target = os.readlink(meshes_dst)
            if target != meshes_src:
                os.unlink(meshes_dst)
        else:
            shutil.rmtree(meshes_dst)
    if not os.path.exists(meshes_dst):
        try:
            os.symlink(meshes_src, meshes_dst)
        except FileExistsError:
            pass

def generate_launch_description():
    pkg_name = 'auwo_excavator_description'
    pkg_share = get_package_share_directory(pkg_name)

    _register_local_model(pkg_share)

    # Local world (avoid Fuel/network)
    default_world = os.path.join(pkg_share, 'worlds', 'empty.sdf')
    world_file = _ensure_local_world(default_world)

    # ---- Args ----
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')  # path to urdf/xacro
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    physics_engine = LaunchConfiguration('physics_engine')  # selector

    # Spawn pose args
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_R = LaunchConfiguration('spawn_R')
    spawn_P = LaunchConfiguration('spawn_P')
    spawn_Y = LaunchConfiguration('spawn_Y')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to local .sdf/.world'
    )
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([FindPackageShare(pkg_name), 'urdf', 'excavator.urdf.xacro']),
        description='Path to top-level Xacro/URDF'
    )
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='excavator')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')

    physics_engine_arg = DeclareLaunchArgument(
        'physics_engine',
        default_value='gz-physics-bullet-featherstone-plugin',
        description='Physics engine plugin (e.g., gz-physics-bullet-featherstone-plugin or gz-physics-dartsim-plugin)'
    )

    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='1.5')
    spawn_R_arg = DeclareLaunchArgument('spawn_R', default_value='0.0')
    spawn_P_arg = DeclareLaunchArgument('spawn_P', default_value='0.0')
    spawn_Y_arg = DeclareLaunchArgument('spawn_Y', default_value='0.0')

    # Resource paths for Gazebo
    user_models = os.path.join(os.path.expanduser("~"), ".gz", "models")
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            TextSubstitution(text=':' + pkg_share),
            TextSubstitution(text=':' + os.path.join(pkg_share, 'meshes')),
            TextSubstitution(text=':' + os.path.join(pkg_share, 'worlds')),
            TextSubstitution(text=':' + user_models),
        ]
    )

    # Build /robot_description from Xacro
    xacro_exec = FindExecutable(name='xacro')
    robot_description_cmd = Command([
        xacro_exec, TextSubstitution(text=' '),
        model,       TextSubstitution(text=' '),
        TextSubstitution(text='use_sim:=true')
    ])
    robot_description = ParameterValue(robot_description_cmd, value_type=str)

    # robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,   # your xacro output
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0,                # <- force TF at 50 Hz
            'ignore_timestamp': True                  # <- don’t be picky about stamps
        }]
    )


    # Gazebo server/gui
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-v', '4',
             '--physics-engine', physics_engine,
             world],
        output='screen',
        condition=IfCondition(headless)
    )
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4',
             '--physics-engine', physics_engine,
             world],
        output='screen',
        condition=UnlessCondition(headless)
    )

    # Spawn robot from /robot_description
    spawner = Node(
        package='ros_gz_sim',
        executable='create',
        name='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
            '-R', spawn_R, '-P', spawn_P, '-Y', spawn_Y,
        ]
    )

    # Bridge Gazebo time -> ROS /clock by REMAPPING the world-scoped clock to /clock
    # This guarantees all ROS nodes using use_sim_time will receive sim time on /clock.
    clock_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/world/default/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]',
            '--ros-args', '-r', '/world/default/clock:=/clock'
        ],
        output='screen'
    )

    # ---- Auto-spawn controllers after Gazebo is up ----
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'config',
        'controllers.yaml'
    ])

    spawner_jsb = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_yaml
        ],
        output='screen'
    )

    spawner_arm = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'arm_position_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_yaml
        ],
        output='screen'
    )

    # Delay a bit so /controller_manager exists before spawning
    spawn_after_gz = TimerAction(period=3.0, actions=[spawner_jsb, spawner_arm])

    return LaunchDescription([
        world_arg, model_arg, robot_name_arg, use_sim_time_arg, headless_arg,
        physics_engine_arg,
        spawn_x_arg, spawn_y_arg, spawn_z_arg, spawn_R_arg, spawn_P_arg, spawn_Y_arg,
        set_gz_resource_path,
        rsp,
        gz_server, gz_gui,
        spawner,
        clock_bridge,          # <-- remapped world clock -> /clock
        spawn_after_gz,        # auto-spawn controllers
    ])

