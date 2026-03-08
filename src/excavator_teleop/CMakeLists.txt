cmake_minimum_required(VERSION 3.16)
project(excavator_teleop)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/teleop_excavator.py
  scripts/trajectory_command_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

