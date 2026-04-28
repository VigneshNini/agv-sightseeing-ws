cmake_minimum_required(VERSION 3.8)
project(agv_safety)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  agv_safety/safety_arbiter_node.py
  agv_safety/emergency_stop_node.py
  agv_safety/collision_monitor_node.py
  agv_safety/system_watchdog_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
