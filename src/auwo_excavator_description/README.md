
# auwo_excavator_description

Now uses **excavator.urdf.xacro** with parameterized mesh paths and auto-added collision geometry.

## Build
```
mkdir -p ~/ros2_ws/src
cp -r auwo_excavator_description ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash
```

## RViz2
```
ros2 launch auwo_excavator_description view_rviz.launch.py
```
Pass a custom mesh directory if needed:
```
ros2 launch auwo_excavator_description view_rviz.launch.py mesh_uri:=package://auwo_excavator_description/meshes
```

## Gazebo (gz)
```
ros2 launch auwo_excavator_description gazebo.launch.py
```
You can provide a custom world via `gz_args`.

## Notes
- All `<mesh filename="...">` entries are rewritten to use `${mesh_uri}/<file>`.
- If a link had no collision, a collision was added using the same mesh. Replace with primitives if you want faster simulation.
- Next: we can add `urdf/sensors/` xacros for LiDAR/Camera and include them from `excavator.urdf.xacro`.
