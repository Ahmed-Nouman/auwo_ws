# AUWO Digital Twin Workspace (Centria)

ROS 2 workspace for the **AUWO** project digital twin: excavator + dump truck in Gazebo Harmonic, with RViz visualization, trajectory-smoothed control, and optional sensors (IMU, LiDAR). This repository reflects **Centria-side** development.

**Maintainer:** Ahmed Nouman (GitHub: [@AhmedNouman](https://github.com/AhmedNouman) — update with your username if different)

## Overview

- **Excavator**: 4-DoF (body, boom, stick, bucket), controlled via **arm_trajectory_controller** (JointTrajectoryController) with a trajectory adapter for smooth motion. **IMU** and **3D LiDAR** (gpu_lidar) sensors on the body; topics `/imu` and `/points` bridged from Gazebo.
- **Dump truck**: Static model (Pete-style) for dump target visualization.
- **Environment**: Ground plane, sun, and Pete environment mesh in `excavation_site_local.sdf`.

## Quick start

```bash
cd /home/ra2/Devel/auwo_ws
colcon build
source install/setup.bash
ros2 launch excavator_interactive_rviz auwo_twin.launch.py
```

- **Gazebo**: Server loads the **excavation site** world by default (`excavation_site_local.sdf`: ground, sun, and terrain mesh from the pete-driving-articulated-dump-truck source). Excavator and truck spawn after ~6 s. For a plain world without terrain, run with `world:=/path/to/empty.sdf` (path from `ros2 pkg prefix excavator_description`).
- **RViz**: Fixed frame `world`; RobotModel displays for Excavator and Truck (topics `/excavator_robot_description`, `/truck_robot_description`).
- **Control**: Use the **Preset Poses** panel: enable motion, choose Simulation, then **Idle** / **Dig** / **Dump** / **Transport**, or **Run Cycle**. Interactive markers also available. Commands go through the trajectory adapter for smooth motion.
- **Sensors**: `/imu` (sensor_msgs/Imu), `/points` (sensor_msgs/PointCloud2 from 3D LiDAR). See **Visualizing sensors in RViz** below.

## Visualizing sensors in RViz

With `view.rviz` (loaded by the digital twin launch):

- **3D LiDAR**: Display **"3D LiDAR"** (PointCloud2). Topic: `/points`. Shows the excavator’s 3D scan. If nothing appears, ensure the excavator is spawned in Gazebo and the bridge is running; you can set the topic’s **Reliability** to **Reliable** if needed.
- **IMU orientation**: Display **"IMU orientation"** (Pose). Topic: `/imu_pose`. Shows the excavator’s orientation as axes (published by `imu_to_pose` from `/imu`). Position is fixed (e.g. 0.5, 0, 1.5 in `world`); the axes rotate with the IMU quaternion.

Both displays are enabled by default. To add them manually: **Add** → **By display type** → **PointCloud2** (topic `/points_viz`) or **Pose** (topic `/imu_pose`). Fixed frame: `world`.

**If `/imu` or `/points` have no data:** (1) **Press Play** in the Gazebo window. (2) The launch sets `GZ_PARTITION=auwo_sim` so the bridge and gz sim share the same transport partition; the bridge maps gz `/excavator/imu` and `/excavator/points` to ROS `/imu` and `/points`. (3) To verify gz data from another terminal, use the same partition: `GZ_PARTITION=auwo_sim gz topic -e -t /excavator/imu`.

## Packages

| Package | Role |
|--------|-----|
| `excavator_description` | URDF/xacro, meshes, world SDF, controller config, RViz config |
| `excavator_gazebo` | Gazebo launch, robot spawn, clock/sensor bridge, publish_robot_descriptions |
| `excavator_interactive_rviz` | RViz launch, interactive markers, twin router |
| `excavator_teleop` | Teleop script |
| `excavator_cycle` | Automatic dig/dump cycle (publishes target positions) |

## Daily / timeline log (Centria)

*(Keep this section updated as you work; add new entries at the top.)*

### 2026-03-08 (trajectory controller + IMU + 3D LiDAR)

- **Trajectory controller**: Added `arm_trajectory_controller` (JointTrajectoryController) to `controllers.yaml`; Gazebo launch spawns it instead of the position controller. `trajectory_command_adapter` node launched from `auwo_twin.launch.py` converts `/arm_position_controller/commands` to smooth `JointTrajectory` for the trajectory controller.
- **IMU**: `sensor_imu_link` and `body_to_imu` joint on excavator; Gazebo `<sensor type="imu">` (100 Hz). Bridge: Gazebo IMU → `/imu` (sensor_msgs/Imu). `imu_to_pose` node publishes `/imu_pose` (PoseStamped) for RViz.
- **3D LiDAR**: `sensor_lidar_link` and `body_to_lidar` joint; Gazebo `<sensor type="gpu_lidar">` (360×32 samples, 30 m range). Bridge: Gazebo PointCloudPacked → `/points` (sensor_msgs/PointCloud2).

### 2026-03-08 (rollback + Gazebo server-first)

- **Rollback**: World set back to `empty.sdf` (start-of-session state).
- **Gazebo empty fix**: Launch now starts the **Gazebo server** with the world file (`gz sim -s ... world.sdf`) so the world loads immediately, then starts the **GUI** after 3 s (`gz sim -g`). Spawn and clock bridge run after 6 s so `/world/default/create` exists; controllers after 9 s. This avoids the GUI-first mode where nothing appears until the user selects a world.

### 2026-03-08 (earlier)

- **Smooth motion**: Switched from `arm_position_controller` to `arm_trajectory_controller`. Added `trajectory_command_adapter` in `excavator_teleop`: subscribes to `/arm_position_controller/commands`, builds smooth JointTrajectory (0.6 s, 15 points, ease-in-out), publishes to `arm_trajectory_controller/joint_trajectory`. Excavation cycle and RViz markers unchanged; motion is less jerky.
- **Documentation**: Added this README with project overview, quick start, package list, smooth-motion and sensor notes, and this daily timeline (Centria responsibility).
- **Sensors**: Added IMU and LiDAR to excavator URDF: links `sensor_imu_link`, `sensor_lidar_link` (fixed to body); Gazebo sensor blocks for IMU (50 Hz) and ray (2D scan, 10 Hz). World `excavation_site.sdf` loads Imu system plugin. Bridge config in `clock_bridge.yaml` extended to bridge `excavator/imu` → `/imu` and `excavator/scan` → `/scan`.

### Earlier (summary)

- Digital twin launch: Gazebo (excavation_site world) + RViz + dump truck; excavator and truck spawn; robot descriptions published to dedicated topics for RViz.
- Dump truck: Pete-style GLB mesh, scaling/orientation adjusted; truck at (4, 3) to avoid overlap with excavator.
- Excavation cycle: Wall-time state machine (PREPARE_DIG → SCOOP_AND_SLEW → LIFT_AND_ALIGN_DUMP → DUMP → RETURN_HOME); warmup 15 s; publishes target positions at 10 Hz.
- Environment: Pete environment mesh (untitled.stl) in world; previous pit model removed.
- Controllers: joint_state_broadcaster + arm_trajectory_controller; clock bridged for sim time; publish_robot_descriptions ensures `/robot_description` is excavator (for gz_ros_control) and RViz topics are fed.

---

*Responsible: Centria side (Ahmed Nouman). Electric excavator and dump truck hardware expected in 2–3 months; digital twin and trajectory/sensor setup continue in the meantime.*
