cmake_minimum_required(VERSION 3.8)
project(agv_dashboard)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  agv_dashboard/web_bridge_node.py
  agv_dashboard/telemetry_logger_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY dashboard grafana
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
