cmake_minimum_required(VERSION 3.16)
project(excavator_description)

find_package(ament_cmake REQUIRED)

# Install excavator URDF only (truck moved to truck_description package).
install(FILES urdf/excavator.urdf.xacro
  DESTINATION share/${PROJECT_NAME}/urdf)

# Install meshes except truck meshes (moved to truck_description).
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
  PATTERN "truck" EXCLUDE
  PATTERN "truck/*" EXCLUDE)

# Install remaining resources as-is.
foreach(dir IN ITEMS config launch worlds models scripts)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir})
  endif()
endforeach()

ament_package()

