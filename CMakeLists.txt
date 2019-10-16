cmake_minimum_required(VERSION 2.8.3)
project(theta_star_2d)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  tf
  costmap_2d
  message_generation
  geometry_msgs
  trajectory_msgs
  actionlib
  genmsg
  actionlib_msgs
)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES ExecutePath.action
)
generate_dynamic_reconfigure_options(
  cfg/LocalPlanner.cfg
  cfg/GlobalPlanner.cfg
  cfg/sim.cfg
)

add_service_files(
  FILES
  GoalCmd.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
  actionlib_msgs

)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime actionlib_msgs
)
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${Boost_INCLUDE_DIRS}
)

add_executable(sim_planner_node src/sim_planner_node.cpp src/ThetaStar.cpp)
target_link_libraries(sim_planner_node ${catkin_LIBRARIES})
add_dependencies(sim_planner_node ${PROJECT_NAME}_gencfg)
add_dependencies(sim_planner_node ${catkin_EXPORTED_TARGETS})

add_executable(global_planner_node src/global_planner_node.cpp src/GlobalPlanner.cpp src/ThetaStar.cpp)
target_link_libraries(global_planner_node ${catkin_LIBRARIES})
add_dependencies(global_planner_node ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${actionlib_tutorials_EXPORTED_TARGETS})

add_executable(local_planner_node src/local_planner_node.cpp src/LocalPlanner.cpp src/ThetaStar.cpp)
target_link_libraries(local_planner_node ${catkin_LIBRARIES})
add_dependencies(local_planner_node ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${actionlib_tutorials_EXPORTED_TARGETS})

add_executable(custom_costmap_node src/custom_costmap_node.cpp)
target_link_libraries(custom_costmap_node ${catkin_LIBRARIES})
add_dependencies(local_planner_node ${catkin_EXPORTED_TARGETS})


add_executable(action_goal_interface_node src/action_goal_interface_node.cpp)

target_link_libraries( action_goal_interface_node
  ${catkin_LIBRARIES}
)

add_dependencies(action_goal_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})