cmake_minimum_required(VERSION 3.16)
project(auwo_excavator_description)

find_package(ament_cmake REQUIRED)

# Your robust loop (keep it)
foreach(dir IN ITEMS urdf meshes config launch worlds models)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

# NEW: also install a copy of meshes under the Gazebo model folder
# so model://auwo_excavator_description/meshes/... resolves.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
  install(DIRECTORY meshes/
    DESTINATION share/${PROJECT_NAME}/models/auwo_excavator_description/meshes)
endif()

ament_package()

install(PROGRAMS
  scripts/teleop_excavator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/excavator_cycle.py
  DESTINATION lib/${PROJECT_NAME}
)


