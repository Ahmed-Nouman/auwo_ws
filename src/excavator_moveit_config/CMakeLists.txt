cmake_minimum_required(VERSION 3.16)
project(excavator_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/auwo_debug_session_probe.py
    scripts/publish_truck_collision_object.py
    scripts/wait_for_arm_trajectory_action.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES .setup_assistant package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(FILES resource/excavator_moveit_config
  DESTINATION share/ament_index/resource_index/packages
)

ament_package()
