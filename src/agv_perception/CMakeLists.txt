cmake_minimum_required(VERSION 3.8)
project(agv_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  agv_perception/lidar_obstacle_detector.py
  agv_perception/object_tracker_node.py
  agv_perception/camera_perception_node.py
  agv_perception/driveable_surface_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
