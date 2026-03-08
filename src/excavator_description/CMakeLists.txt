cmake_minimum_required(VERSION 3.16)
project(excavator_description)

find_package(ament_cmake REQUIRED)

# Install description resources
foreach(dir IN ITEMS urdf meshes config launch worlds models scripts)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir})
  endif()
endforeach()

ament_package()

