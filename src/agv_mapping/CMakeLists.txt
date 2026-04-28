cmake_minimum_required(VERSION 3.8)
project(agv_mapping)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  agv_mapping/ndt_mapping_node.py
  agv_mapping/octomap_node.py
  agv_mapping/map_server_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
