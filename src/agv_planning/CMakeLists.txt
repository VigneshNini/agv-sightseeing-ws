cmake_minimum_required(VERSION 3.8)
project(agv_planning)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  agv_planning/global_route_manager.py
  agv_planning/hybrid_astar_planner.py
  agv_planning/mpc_local_planner.py
  agv_planning/velocity_profiler.py
  agv_planning/behavior_tree_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
