# excavator_description

URDF/xacro for the excavator (and truck), meshes, world SDF, controller config, and RViz config. Uses **excavator.urdf.xacro** with parameterized mesh paths and collision geometry.

## Build
```
mkdir -p ~/ros2_ws/src
cp -r excavator_description ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash
```

## RViz2
```
ros2 launch excavator_description view_rviz.launch.py
```
Pass a custom mesh directory if needed:
```
ros2 launch excavator_description view_rviz.launch.py mesh_uri:=package://excavator_description/meshes
```

## Gazebo (gz)
```
ros2 launch excavator_description gazebo.launch.py
```
You can provide a custom world via `gz_args`.

## Dump truck and excavation environment
- **Model:** `urdf/truck.urdf.xacro` — dump truck with mesh (from `jsk_mbzirc_common` or vendored in `meshes/truck/`), plus `dump_bed` and `dump_target` frames for the excavator. Set property `mesh_uri` in the xacro if you copy meshes into `excavator_description/meshes/truck/`.
- **World:** `worlds/excavation_site.sdf` — ground, sun, and excavation environment (dirt mounds). Used by default when launching the digital twin with excavation.
- **Launch:** `ros2 launch excavator_interactive_rviz auwo_twin.launch.py` (excavation + truck by default). Set RViz **Fixed Frame** to `world` to see excavator and truck. Use `use_excavation_site:=false` for empty world without truck.

## Notes
- All `<mesh filename="...">` entries are rewritten to use `${mesh_uri}/<file>`.
- If a link had no collision, a collision was added using the same mesh. Replace with primitives if you want faster simulation.
- Next: we can add `urdf/sensors/` xacros for LiDAR/Camera and include them from `excavator.urdf.xacro`.
