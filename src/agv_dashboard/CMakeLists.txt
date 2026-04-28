cmake_minimum_required(VERSION 3.8)
project(agv_dashboard)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY dashboard grafana
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
