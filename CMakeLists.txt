cmake_minimum_required(VERSION 2.8.3)
project(theta_star_2d)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

if($ENV{ROS_DISTRO} STREQUAL "melodic" )
add_definitions(-DMELODIC)
endif()

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  upo_actions
  roscpp
  costmap_2d
  geometry_msgs
  trajectory_msgs
  octomap_msgs
  octomap_ros
  pcl_conversions
  pcl_ros
  marsupial_g2o
)
find_package(Boost REQUIRED COMPONENTS system)

add_service_files(
  FILES
  SaveMission.srv
  checkObstacles.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/LocalPlanner.cfg
  cfg/GlobalPlanner.cfg
  cfg/sim.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp upo_actions message_runtime octomap_msgs octomap_ros pcl_conversions pcl_ros marsupial_g2o
  DEPENDS octomap_msgs octomap_ros
  LIBRARIES ${EXTERNAL_LIBS} 
)
###########
## Build ##
###########

include_directories(include/
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${Boost_INCLUDE_DIRS}
)

#add_executable(sim_planner_node src/sim_planner_node.cpp src/ThetaStar.cpp)
#target_link_libraries(sim_planner_node ${catkin_LIBRARIES})
#add_dependencies(sim_planner_node ${PROJECT_NAME}_gencfg)
#add_dependencies(sim_planner_node ${catkin_EXPORTED_TARGETS})

add_executable(global_planner_node src/global_planner_node.cpp src/GlobalPlanner.cpp src/ThetaStar2D.cpp src/ThetaStar3D.cpp )
target_link_libraries(global_planner_node  marsupial_g2o ${catkin_LIBRARIES} ${EXTERNAL_LIBS})
add_dependencies(global_planner_node ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(local_planner_node src/local_planner_node.cpp src/LocalPlanner.cpp src/ThetaStar2D.cpp src/ThetaStar3D.cpp )
target_link_libraries(local_planner_node marsupial_g2o ${catkin_LIBRARIES} ${EXTERNAL_LIBS})
add_dependencies(local_planner_node ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(custom_costmap_node src/custom_costmap_node.cpp)
target_link_libraries(custom_costmap_node ${catkin_LIBRARIES})
add_dependencies(custom_costmap_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(action_goal_interface_node src/action_goal_interface_node.cpp)

add_executable(goal_publisher_node src/goal_publisher.cpp)
target_link_libraries(goal_publisher_node ${catkin_LIBRARIES})
add_dependencies(goal_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries( action_goal_interface_node
  ${catkin_LIBRARIES}
)

add_dependencies(action_goal_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(hmi_fake src/hmi_fake.cpp)

target_link_libraries( hmi_fake
  ${catkin_LIBRARIES}
)

add_dependencies(hmi_fake ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})