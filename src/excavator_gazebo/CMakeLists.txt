cmake_minimum_required(VERSION 3.16)
project(excavator_gazebo)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/publish_robot_descriptions.py
  scripts/imu_to_pose.py
  scripts/points_frame_remap.py
  scripts/spawn_controller_safe.sh
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()

