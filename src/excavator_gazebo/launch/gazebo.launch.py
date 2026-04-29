#!/usr/bin/env python3
# Gazebo Harmonic launcher for excavator_description (ROS 2 Jazzy)
import os
import shutil
import subprocess
import tempfile
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    ExecuteProcess,
    TimerAction,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    EnvironmentVariable,
    TextSubstitution,
    Command,
    PathJoinSubstitution,
    FindExecutable,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

# Same transport partition for gz sim, bridges, and ros_gz_sim create (must match).
_GZ_TRANSPORT_ENV = {
    'GZ_PARTITION': 'auwo_sim',
    'IGN_PARTITION': 'auwo_sim',
    # FIX: GZ_SIM_SYSTEM_PLUGIN_PATH was missing from the gz sim server environment.
    # Without it, Gazebo cannot find libgz_ros2_control-system.so even though it
    # exists in /opt/ros/jazzy/lib — Gazebo uses this var, not LD_LIBRARY_PATH,
    # to locate system plugins. This caused gz_ros2_control to silently not load,
    # leaving /controller_manager unreachable and /joint_states unpublished.
    'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib',
}

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
    """Create ~/.gz/models/excavator_description so model:// URIs can resolve locally."""
    home = os.path.expanduser("~")
    models_root = os.path.join(home, ".gz", "models")
    model_root = os.path.join(models_root, "excavator_description")
    meshes_src = os.path.join(pkg_share, "meshes")
    meshes_dst = os.path.join(model_root, "meshes")

    os.makedirs(models_root, exist_ok=True)
    os.makedirs(model_root, exist_ok=True)

    config_path = os.path.join(model_root, "model.config")
    if not os.path.exists(config_path):
        with open(config_path, "w") as f:
            f.write("""<?xml version="1.0"?>
<model>
  <name>excavator_description</name>
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
  <model name="excavator_description">
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


def _write_rviz_description_files(
    excavator_pkg_share: str,
    excavator_xacro: str,
    truck_pkg_share: str,
    _truck_content_unused: str | None,
) -> tuple[str, str]:
    """Write excavator and truck URDF to temp files for RViz (keep package:// so RViz resolves meshes). Returns (excavator_path, truck_path)."""
    out_excavator = os.path.join(tempfile.gettempdir(), 'excavator_rviz.urdf')
    result = subprocess.run(
        ['xacro', excavator_xacro, 'use_sim:=true', '-o', out_excavator],
        capture_output=True,
        text=True,
        timeout=10,
    )
    if result.returncode != 0:
        raise RuntimeError(f"xacro excavator for RViz failed: {result.stderr or result.stdout}")

    # SIM-ONLY: Zero base_to_base_link rpy in RViz URDF too.
    # RSP uses this file to publish TF. If it contains rpy="0 0 3.14159",
    # the 180° propagates through the kinematic chain causing -180° roll on
    # the bucket frame. With rpy="0 0 0" here AND spawn_Y=0 on the static TF,
    # all frames are consistent with Gazebo physics.
    with open(out_excavator, 'r') as _f:
        _rviz_content = _f.read()
    _marker = 'name="base_to_base_link"'
    if _marker in _rviz_content:
        _start = _rviz_content.index(_marker)
        _end = _rviz_content.index('</joint>', _start) + len('</joint>')
        _block = _rviz_content[_start:_end]
        import re as _re2
        _fixed = _re2.sub(r'rpy="[^"]*"', 'rpy="0 0 0"', _block)
        _rviz_content = _rviz_content[:_start] + _fixed + _rviz_content[_end:]
        with open(out_excavator, 'w') as _f:
            _f.write(_rviz_content)

    # Also zero the bucket visual rpy="0 3.14159 0".
    # That pitch flip was authored to compensate for the base_to_base_link
    # 180° yaw in the physical twin. With base_to_base_link now zeroed for
    # sim, the bucket mesh appears upside down without this fix.
    _bmarker = '<link name="bucket">'
    if _bmarker in _rviz_content:
        _bs = _rviz_content.index(_bmarker)
        _be = _rviz_content.index('</link>', _bs) + len('</link>')
        _bblock = _rviz_content[_bs:_be]
        # Only zero visual origin rpys, not collision
        _bparts = _bblock.split('<visual>')
        for _bi in range(1, len(_bparts)):
            _vend = _bparts[_bi].index('</visual>')
            _visual = _bparts[_bi][:_vend]
            _visual = _re2.sub(r'rpy="[^"]*"', 'rpy="0 0 0"', _visual)
            _bparts[_bi] = _visual + _bparts[_bi][_vend:]
        _bblock_fixed = '<visual>'.join(_bparts)
        _rviz_content = _rviz_content[:_bs] + _bblock_fixed + _rviz_content[_be:]
        with open(out_excavator, 'w') as _f:
            _f.write(_rviz_content)

    out_truck = os.path.join(tempfile.gettempdir(), 'truck_rviz.urdf')
    truck_xacro = os.path.join(truck_pkg_share, 'urdf', 'truck.urdf.xacro')
    if os.path.isfile(truck_xacro):
        result = subprocess.run(
            ['xacro', truck_xacro, 'pete_visual_ext:=stl', 'pete_visual_rpy:=0 0 0', '-o', out_truck],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if result.returncode != 0:
            with open(out_truck, 'w') as f:
                f.write('<?xml version="1.0"?><robot name="truck"><link name="base_link"><visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual></link></robot>')
        # else: xacro wrote out_truck with package:// so RViz can resolve meshes
    else:
        with open(out_truck, 'w') as f:
            f.write('<?xml version="1.0"?><robot name="truck"><link name="base_link"><visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual></link></robot>')
    return out_excavator, out_truck


def _generate_truck_urdf(truck_pkg_share: str) -> tuple[str, str]:
    """Run xacro on truck.urdf.xacro; return (path_to_urdf_file, urdf_content)."""
    xacro_path = os.path.join(truck_pkg_share, 'urdf', 'truck.urdf.xacro')
    if not os.path.isfile(xacro_path):
        raise FileNotFoundError(f"Truck xacro not found: {xacro_path}")
    out_path = os.path.join(tempfile.gettempdir(), 'truck_excavation_site.urdf')
    result = subprocess.run(
        ['xacro', xacro_path, '-o', out_path],
        capture_output=True,
        text=True,
        timeout=10,
    )
    if result.returncode != 0:
        raise RuntimeError(f"xacro failed: {result.stderr or result.stdout}")
    with open(out_path, 'r') as f:
        content = f.read()
    # Resolve package:// so Gazebo finds meshes
    pkg_share = os.path.dirname(os.path.dirname(xacro_path))  # .../share/truck_description
    content = content.replace(
        'package://truck_description/',
        pkg_share.rstrip('/') + '/',
    )
    with open(out_path, 'w') as f:
        f.write(content)
    return out_path, content


def _generate_excavator_urdf_resolved(pkg_share: str, model_path: str) -> tuple[str, str]:
    """Run xacro on excavator, resolve package:// to absolute paths; return (file_path, content)."""
    if not os.path.isfile(model_path):
        raise FileNotFoundError(f"Excavator xacro not found: {model_path}")
    out_path = os.path.join(tempfile.gettempdir(), 'excavator_resolved.urdf')
    result = subprocess.run(
        ['xacro', model_path, 'use_sim:=true', '-o', out_path],
        capture_output=True,
        text=True,
        timeout=10,
    )
    if result.returncode != 0:
        raise RuntimeError(f"xacro failed: {result.stderr or result.stdout}")
    with open(out_path, 'r') as f:
        content = f.read()
    # Replace package:// so Gazebo can find meshes without package resolution
    content = content.replace(
        'package://excavator_description/',
        pkg_share.rstrip('/') + '/',
    )

    # SIM-ONLY FIX: Zero out the rpy on base_to_base_link fixed joint.
    # The URDF has rpy="0 0 3.14159" there for the physical twin, but in
    # simulation ros_gz_sim spawns the physics body directly in world frame
    # without composing through fixed joints. Keeping the 180° yaw causes it
    # to propagate through the kinematic chain, adding a spurious 180° roll
    # to all child frames (visible as 180° roll on bucket in tf2_echo output).
    # We patch it out here in the Gazebo-only resolved URDF using simple string
    # replacement — the source URDF on disk is never modified.
    # Find the base_to_base_link joint block and replace its origin rpy only.
    _marker = 'name="base_to_base_link"'
    if _marker in content:
        _start = content.index(_marker)
        _end = content.index('</joint>', _start) + len('</joint>')
        _joint_block = content[_start:_end]
        # Replace whatever rpy is in the origin tag within this joint block
        import re as _re
        _fixed = _re.sub(r'rpy="[^"]*"', 'rpy="0 0 0"', _joint_block)
        content = content[:_start] + _fixed + content[_end:]


        # Also zero the bucket visual rpy="0 3.14159 0".
        # That pitch flip was authored to compensate for the base_to_base_link
        # 180° yaw in the physical twin. With base_to_base_link now zeroed for
        # sim, the bucket mesh appears upside down without this fix.
        _bmarker = '<link name="bucket">'
        if _bmarker in content:
            _bs = content.index(_bmarker)
            _be = content.index('</link>', _bs) + len('</link>')
            _bblock = content[_bs:_be]
            # Only zero visual origin rpys, not collision
            _bparts = _bblock.split('<visual>')
            for _bi in range(1, len(_bparts)):
                _vend = _bparts[_bi].index('</visual>')
                _visual = _bparts[_bi][:_vend]
                _visual = _re.sub(r'rpy="[^"]*"', 'rpy="0 0 0"', _visual)
                _bparts[_bi] = _visual + _bparts[_bi][_vend:]
            _bblock_fixed = '<visual>'.join(_bparts)
            content = content[:_bs] + _bblock_fixed + content[_be:]
    with open(out_path, 'w') as f:
        f.write(content)
    return out_path, content


def generate_launch_description():
    pkg_name = 'excavator_description'
    pkg_share = get_package_share_directory(pkg_name)
    pkg_gazebo_share = get_package_share_directory('excavator_gazebo')
    truck_pkg_share = get_package_share_directory('truck_description')

    _register_local_model(pkg_share)

    # Generate truck URDF once so spawn_dumper (spawn truck) can use it
    try:
        truck_urdf_path, truck_urdf_content = _generate_truck_urdf(truck_pkg_share)
    except (FileNotFoundError, RuntimeError):
        truck_urdf_path = None
        truck_urdf_content = None

    # Temp files with URDF for RViz (excavator keeps package:// so RViz resolves meshes)
    excavator_xacro = os.path.join(pkg_share, 'urdf', 'excavator.urdf.xacro')
    try:
        excavator_rviz_path, truck_rviz_path = _write_rviz_description_files(
            pkg_share, excavator_xacro, truck_pkg_share, truck_urdf_content or ''
        )
    except (FileNotFoundError, RuntimeError) as e:
        excavator_rviz_path = os.path.join(tempfile.gettempdir(), 'excavator_rviz.urdf')
        truck_rviz_path = os.path.join(tempfile.gettempdir(), 'truck_rviz.urdf')
        # Ensure files exist so publish_robot_descriptions can read something
        if not os.path.isfile(excavator_rviz_path) and os.path.isfile(excavator_xacro):
            subprocess.run(['xacro', excavator_xacro, 'use_sim:=true', '-o', excavator_rviz_path],
                          capture_output=True, timeout=10)
        if not os.path.isfile(truck_rviz_path):
            truck_xacro = os.path.join(truck_pkg_share, 'urdf', 'truck.urdf.xacro')
            if os.path.isfile(truck_xacro):
                subprocess.run(
                    ['xacro', truck_xacro, 'pete_visual_ext:=stl', 'pete_visual_rpy:=0 0 0', '-o', truck_rviz_path],
                    capture_output=True,
                    timeout=10,
                )

    # Excavator URDF with resolved package:// for Gazebo when spawning with truck
    excavator_model = excavator_xacro
    try:
        excavator_urdf_path, _ = _generate_excavator_urdf_resolved(pkg_share, excavator_model)
    except (FileNotFoundError, RuntimeError):
        excavator_urdf_path = None

    # Local world (avoid Fuel/network)
    default_world = os.path.join(pkg_share, 'worlds', 'empty.sdf')
    world_file = _ensure_local_world(default_world)

    # ---- Args ----
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')  # path to urdf/xacro
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    unified_gui = LaunchConfiguration('gazebo_unified_gui')
    gazebo_verbose = LaunchConfiguration('gazebo_verbose')
    physics_engine = LaunchConfiguration('physics_engine')  # selector
    spawn_dumper = LaunchConfiguration('spawn_dumper')

    # Spawn pose args
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_R = LaunchConfiguration('spawn_R')
    spawn_P = LaunchConfiguration('spawn_P')
    spawn_Y = LaunchConfiguration('spawn_Y')

    dumper_x = LaunchConfiguration('dumper_x')
    dumper_y = LaunchConfiguration('dumper_y')
    dumper_z = LaunchConfiguration('dumper_z')
    dumper_yaw = LaunchConfiguration('dumper_yaw')
    # Keep a safe fallback so this launch still works even if a parent launch
    # forgets to declare/forward this argument.
    controller_spawn_delay_sec = LaunchConfiguration('controller_spawn_delay_sec', default='25.0')

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

    gazebo_unified_gui_arg = DeclareLaunchArgument(
        'gazebo_unified_gui',
        default_value='false',
        description=(
            'If true (and headless:=false), run one gz sim process (GUI+server). '
            'If false, use gz sim -s then gz sim -g (default; reliable ros_gz_sim create).'
        ),
    )
    gazebo_verbose_arg = DeclareLaunchArgument(
        'gazebo_verbose',
        default_value='1',
        description='Gazebo log verbosity for gz sim -v (0–4). Lower = less console spam.',
    )

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

    spawn_dumper_arg = DeclareLaunchArgument(
        'spawn_dumper',
        default_value='false',
        description='Spawn dump truck model (truck.urdf.xacro) for excavation scenario',
    )
    # Default truck pose: spaced apart from excavator to avoid overlap
    dumper_x_arg = DeclareLaunchArgument('dumper_x', default_value='4.0')
    dumper_y_arg = DeclareLaunchArgument('dumper_y', default_value='3.0')
    dumper_z_arg = DeclareLaunchArgument('dumper_z', default_value='0.5')
    dumper_yaw_arg = DeclareLaunchArgument('dumper_yaw', default_value='0.0')

    controller_spawn_delay_sec_arg = DeclareLaunchArgument(
        'controller_spawn_delay_sec',
        default_value='25.0',  # FIX: was 12.0 — plugin now loads correctly but needs more time to initialize /controller_manager
        description=(
            'Seconds after Gazebo starts before spawning joint_state_broadcaster and '
            'arm_trajectory_controller (allow model + /clock bridge to be ready).'
        ),
    )

    # Same Gazebo Transport partition for gz sim + bridge so /excavator/imu and /excavator/points are visible to the bridge
    set_gz_partition = SetEnvironmentVariable(name='GZ_PARTITION', value='auwo_sim')

    # FIX: Set GZ_SIM_SYSTEM_PLUGIN_PATH so Gazebo can find libgz_ros2_control-system.so.
    # Gazebo Harmonic resolves system plugins via this env var, not LD_LIBRARY_PATH.
    # Without it the gz_ros2_control plugin silently fails to load → no /controller_manager.
    set_gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value=''),
            TextSubstitution(text=':/opt/ros/jazzy/lib'),
        ]
    )

    # Resource paths for Gazebo (package://excavator_description/... needs share parent)
    user_models = os.path.join(os.path.expanduser("~"), ".gz", "models")
    share_parent = os.path.dirname(pkg_share)  # so package://excavator_description/meshes resolves
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            TextSubstitution(text=':' + share_parent),
            TextSubstitution(text=':' + pkg_share),
            TextSubstitution(text=':' + os.path.join(pkg_share, 'meshes')),
            TextSubstitution(text=':' + os.path.join(pkg_share, 'models')),
            TextSubstitution(text=':' + os.path.join(pkg_share, 'worlds')),
            TextSubstitution(text=':' + user_models),
        ]
    )

    # RSP uses the patched RViz URDF (base_to_base_link rpy="0 0 0") so TF
    # chain is consistent with Gazebo physics — eliminates -180° roll on bucket.
    with open(excavator_rviz_path, 'r') as _f:
        _rsp_urdf = _f.read()

    # robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': _rsp_urdf,
            'use_sim_time': False,
            'publish_frequency': 50.0,
            'ignore_timestamp': True,
        }]
    )


    # FIX: The excavator URDF has rpy="0 0 3.14159" on the base_to_base_link fixed
    # joint — intentional for the physical twin where localisation compensates.
    # In simulation, ros_gz_sim spawns the physics body at spawn_Y directly and
    # does NOT compose through fixed URDF joints, so the visual mesh ends up 180°
    # from where physics placed it (arm on wrong side).
    #
    # The URDF is left unchanged. Instead the static TF yaw is offset by +3.14159
    # (the same value as the URDF fixed joint) so that RViz resolves all child
    # frames (base, boom, stick, bucket) consistently with Gazebo physics.
    #
    # z=0.0: spawn_z is the drop height; robot settles to ground, TF must match.
    # world -> base_link static TF published at spawn_Y + 3.14159.
    #
    # RViz uses the ORIGINAL URDF (unmodified, with rpy="0 0 3.14159" on
    # base_to_base_link). So the static TF must offset by +3.14159 to make
    # RViz render all child meshes (base, body, boom, stick, bucket) in the
    # correct world orientation.
    #
    # Gazebo uses a SIM-ONLY patched URDF where base_to_base_link rpy="0 0 0"
    # (see _generate_excavator_urdf_resolved). So the Gazebo spawner uses
    # spawn_Y directly (spawn_Y_gz = spawn_Y, no offset needed).
    #
    # Result: both RViz and Gazebo show the robot at the same world orientation,
    # with no spurious 180° roll accumulating through the kinematic chain.
    excavator_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='excavator_world_tf',
        arguments=[
            '--x', spawn_x, '--y', spawn_y, '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0',
            '--yaw', spawn_Y,  # No offset: base_to_base_link rpy is zeroed in sim URDFs
            '--frame-id', 'world', '--child-frame-id', 'base_link',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # spawn_Y_gz == spawn_Y: the 180° base_to_base_link offset is now zeroed
    # in the Gazebo-only resolved URDF (_generate_excavator_urdf_resolved),
    # so no extra rotation is needed at spawn time.
    spawn_Y_gz = spawn_Y

    # Publish excavator and truck URDF to separate topics for RViz (paths as plain strings so they resolve reliably)
    desc_publisher = Node(
        package='excavator_gazebo',
        executable='publish_robot_descriptions.py',
        name='publish_robot_descriptions',
        output='screen',
        parameters=[{
            'excavator_description_file': excavator_rviz_path,
            'truck_description_file': truck_rviz_path,
        }],
    )

    # Split: server (-s) + delayed GUI (-g). Unified: single process (some GPUs/desktops are smoother).
    # No -r: start paused (use Play in the Gazebo GUI to run); -r would auto-run on start.
    gz_server = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-s', '-v', gazebo_verbose,
            '--physics-engine', physics_engine, world,
        ],
        output='screen',
        additional_env=_GZ_TRANSPORT_ENV,
    )
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        additional_env=_GZ_TRANSPORT_ENV,
        condition=UnlessCondition(headless),
    )
    gz_gui_delayed = TimerAction(period=3.0, actions=[gz_gui])
    gz_unified = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-v', gazebo_verbose,
            '--physics-engine', physics_engine, world,
        ],
        output='screen',
        additional_env=_GZ_TRANSPORT_ENV,
    )

    use_unified_gui = PythonExpression(
        [
            "'",
            headless,
            "'.lower() not in ('true', '1') and '",
            unified_gui,
            "'.lower() in ('true', '1')",
        ]
    )
    use_split_gui = PythonExpression(
        [
            "'",
            headless,
            "'.lower() in ('true', '1') or '",
            unified_gui,
            "'.lower() not in ('true', '1')",
        ]
    )
    gz_unified_group = GroupAction(
        actions=[gz_unified],
        condition=IfCondition(use_unified_gui),
    )
    gz_split_group = GroupAction(
        actions=[gz_server, gz_gui_delayed],
        condition=IfCondition(use_split_gui),
    )

    # Spawn excavator from the pre-resolved URDF file (package:// already replaced).
    # Using -file avoids gz transport service-discovery issues that -topic can hit.
    if excavator_urdf_path is not None:
        spawner = Node(
            package='ros_gz_sim',
            executable='create',
            name='create',
            output='screen',
            arguments=[
                '-world', 'default',
                '-file', excavator_urdf_path,
                '-name', robot_name,
                '-allow_renaming', 'true',
                '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
                '-R', spawn_R, '-P', spawn_P, '-Y', spawn_Y_gz,
            ],
            additional_env=_GZ_TRANSPORT_ENV,
        )
    else:
        spawner = Node(
            package='ros_gz_sim',
            executable='create',
            name='create',
            output='screen',
            arguments=[
                '-world', 'default',
                '-topic', 'robot_description',
                '-name', robot_name,
                '-allow_renaming', 'true',
                '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
                '-R', spawn_R, '-P', spawn_P, '-Y', spawn_Y_gz,
            ],
            additional_env=_GZ_TRANSPORT_ENV,
        )

    # Bridge nodes must use use_sim_time:=false so they do not block waiting for /clock while publishing it.
    clock_bridge_config = os.path.join(pkg_gazebo_share, 'config', 'clock_bridge.yaml')
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='bridge_node',
        name='auwo_clock_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': False, 'config_file': clock_bridge_config},
        ],
        additional_env=_GZ_TRANSPORT_ENV,
        arguments=['--ros-args', '--log-level', 'info'],
    )
    sensors_bridge_config = os.path.join(pkg_gazebo_share, 'config', 'sensors_bridge.yaml')
    sensors_bridge = Node(
        package='ros_gz_bridge',
        executable='bridge_node',
        name='auwo_sensor_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': False, 'config_file': sensors_bridge_config},
        ],
        additional_env=_GZ_TRANSPORT_ENV,
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # Spawn immediately — gz transport service discovery requires the create node
    # to join the network alongside the server to catch multicast announcements.
    # The create node retries internally until the service appears.
    bridge_clock_early = TimerAction(period=1.5, actions=[clock_bridge])
    bridge_sensors_delayed = TimerAction(period=5.0, actions=[sensors_bridge])

    # ---- Auto-spawn controllers after spawn has run ----
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
            '--param-file', controllers_yaml,
            '--controller-manager-timeout', '30',
            '--switch-timeout', '15',
        ],
        output='screen'
    )

    spawner_arm = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'arm_trajectory_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_yaml,
            '--controller-manager-timeout', '30',
            '--switch-timeout', '15',
        ],
        output='screen'
    )

    # NOTE: arm_position_controller is declared in controllers.yaml but intentionally
    # NOT spawned here. It is not a hardware controller — /arm_position_controller/commands
    # is just a ROS topic that trajectory_command_adapter subscribes to. Spawning it as a
    # ros2_control controller would conflict with arm_trajectory_controller over the same
    # joints (both claim position command interfaces on the same 4 joints).
    #
    # The command flow is:
    #   twin_router → /arm_position_controller/commands (Float64MultiArray, topic only)
    #   → trajectory_command_adapter → /arm_trajectory_controller/joint_trajectory
    #   → arm_trajectory_controller (the actual active hardware controller)

    # Chain spawners: JSB first, then trajectory controller.
    # Parallel spawners race controller_manager and often never reach ACTIVE.
    after_jsb_spawn_arm = RegisterEventHandler(
        OnProcessExit(target_action=spawner_jsb, on_exit=[spawner_arm]),
    )

    # Controllers after model is in world and /clock bridge is up.
    spawn_after_gz = TimerAction(
        period=controller_spawn_delay_sec,
        actions=[spawner_jsb, after_jsb_spawn_arm],
    )

    # ---- Dump truck (when spawn_dumper is true and truck URDF was generated) ----
    truck_group_actions = []
    if truck_urdf_path is not None and truck_urdf_content is not None:
        truck_spawn = Node(
            package='ros_gz_sim',
            executable='create',
            name='create_truck',
            output='screen',
            arguments=[
                '-world', 'default',
                '-file', truck_urdf_path,
                '-name', 'truck',
                '-allow_renaming', 'true',
                '-x', dumper_x, '-y', dumper_y, '-z', dumper_z,
                '-R', '0', '-P', '0', '-Y', dumper_yaw,
            ],
            additional_env=_GZ_TRANSPORT_ENV,
            condition=IfCondition(spawn_dumper),
        )

        # Stationary dumper: robot_state_publisher publishes the link tree (base_link -> dump_bed) for RViz
        truck_rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_truck',
            output='screen',
            # Do not publish to /truck_robot_description: RViz uses that topic from
            # publish_robot_descriptions (STL). RSP only needs TF; GLB URDF stays off RViz path.
            remappings=[
                ('robot_description', '/truck_robot_description_internal'),
                # FIX: truck RSP was publishing to /joint_states (default topic),
                # colliding with the excavator's joint_state_broadcaster output.
                # Excavator RSP was consuming truck joint data instead of real
                # excavator joints → all arm TF transforms missing in RViz.
                # The truck is static (no moving joints) so its /joint_states
                # output is empty/irrelevant — redirect it to an unused topic.
                ('joint_states', '/truck/joint_states'),
            ],
            parameters=[{
                'robot_description': truck_urdf_content,
                'use_sim_time': use_sim_time,
                'frame_prefix': 'truck/',
                'publish_frequency': 50.0,
            }],
            condition=IfCondition(spawn_dumper),
        )

        # World -> truck/base_link at spawn pose so truck is positioned in RViz (use_sim_time so TF matches /clock)
        truck_static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='truck_world_tf',
            arguments=[
                '--x', dumper_x, '--y', dumper_y, '--z', dumper_z,
                '--yaw', dumper_yaw,
                '--frame-id', 'world', '--child-frame-id', 'truck/base_link',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(spawn_dumper),
        )

        truck_group_actions = [truck_spawn, truck_rsp, truck_static_tf]

    truck_group = GroupAction(
        condition=IfCondition(spawn_dumper),
        actions=truck_group_actions,
    ) if truck_group_actions else None

    # Delay truck-related spawn/actions so excavator spawn from /robot_description is deterministic first.
    truck_group_delayed = TimerAction(period=6.0, actions=[truck_group]) if truck_group is not None else None

    launch_actions = [
        world_arg, model_arg, robot_name_arg, use_sim_time_arg, headless_arg,
        gazebo_unified_gui_arg,
        gazebo_verbose_arg,
        physics_engine_arg,
        spawn_x_arg, spawn_y_arg, spawn_z_arg, spawn_R_arg, spawn_P_arg, spawn_Y_arg,
        spawn_dumper_arg, dumper_x_arg, dumper_y_arg, dumper_z_arg, dumper_yaw_arg,
        controller_spawn_delay_sec_arg,
        set_gz_partition,
        set_gz_resource_path,
        set_gz_plugin_path,
        rsp,
        excavator_world_tf,
        desc_publisher,
        gz_unified_group,
        gz_split_group,
        spawner,
        bridge_clock_early,
        bridge_sensors_delayed,
        spawn_after_gz,
    ]
    if truck_group_delayed is not None:
        launch_actions.append(truck_group_delayed)

    return LaunchDescription(launch_actions)
