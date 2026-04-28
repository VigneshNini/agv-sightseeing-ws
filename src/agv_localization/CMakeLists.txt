cmake_minimum_required(VERSION 3.8)
project(agv_localization)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  agv_localization/ukf_node.py
  agv_localization/lidar_odometry_node.py
  agv_localization/gps_converter_node.py
  agv_localization/wheel_odometry_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
