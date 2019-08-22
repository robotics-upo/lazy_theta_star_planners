cmake_minimum_required(VERSION 2.8.3)
project(theta_star_2d)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  tf
  costmap_2d
)
generate_dynamic_reconfigure_options(
  cfg/sim.cfg
  cfg/local.cfg
  cfg/global.cfg
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES theta_star
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_library(theta_star_2d src/ThetaStar.cpp)

add_executable(sim_planner_node src/sim_planner_node.cpp)
target_link_libraries(sim_planner_node ${catkin_LIBRARIES} theta_star_2d)
add_dependencies(sim_planner_node ${PROJECT_NAME}_gencfg)
add_dependencies(sim_planner_node ${catkin_EXPORTED_TARGETS})

add_executable(global_planner_node src/global_planner_node.cpp src/GlobalPlanner.cpp src/ThetaStar.cpp)
target_link_libraries(global_planner_node ${catkin_LIBRARIES})
add_dependencies(global_planner_node ${PROJECT_NAME}_gencfg)
add_dependencies(global_planner_node ${catkin_EXPORTED_TARGETS})

add_executable(local_planner_node src/local_planner_node.cpp src/LocalPlanner.cpp src/ThetaStar.cpp)
target_link_libraries(local_planner_node ${catkin_LIBRARIES})
add_dependencies(local_planner_node ${PROJECT_NAME}_gencfg)
add_dependencies(local_planner_node ${catkin_EXPORTED_TARGETS})

add_executable(custom_costmap_node src/custom_costmap_node.cpp)
target_link_libraries(custom_costmap_node ${catkin_LIBRARIES})
add_dependencies(local_planner_node ${catkin_EXPORTED_TARGETS})