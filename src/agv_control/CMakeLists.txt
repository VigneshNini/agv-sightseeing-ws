cmake_minimum_required(VERSION 3.8)
project(agv_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  agv_control/lqr_controller.py
  agv_control/can_bus_interface.py
  agv_control/motor_controller_node.py
  agv_control/steering_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
