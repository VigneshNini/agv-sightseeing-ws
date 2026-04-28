cmake_minimum_required(VERSION 3.8)
project(agv_tour_manager)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  agv_tour_manager/tour_manager_node.py
  agv_tour_manager/audio_player_node.py
  agv_tour_manager/passenger_display_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
