cmake_minimum_required(VERSION 3.16)
project(excavator_cycle)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/excavator_cycle.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

