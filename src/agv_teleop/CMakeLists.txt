cmake_minimum_required(VERSION 3.8)
project(agv_teleop)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  agv_teleop/joystick_teleop_node.py
  agv_teleop/keyboard_teleop_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
