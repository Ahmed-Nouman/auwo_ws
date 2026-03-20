# Truck meshes

**Pete articulated dump truck**  
- `source/pete_dump_truck.glb` — dump truck only (Blender-exported, excavation/dirt removed). Used for **Gazebo** spawn (default `pete_visual_ext` in xacro).
- `source/pete_dump_truck.stl` — same truck as STL; used for **RViz** (generated `truck_rviz.urdf` uses `pete_visual_ext:=stl`).
- `textures/` — textures used by the GLB (relative paths from `source/`).
- Used when `use_pete_mesh` is `true` in `truck.urdf.xacro`.

**Legacy (jsk_mbzirc_common)**  
To use the old DAE truck instead, set in `truck.urdf.xacro`:

```xml
<xacro:property name="use_pete_mesh" value="0" />
```

and ensure `jsk_mbzirc_common` is installed, or copy `truck.dae` and `heliport.dae` into this directory and set `mesh_uri` to `package://excavator_description/meshes/truck`.
