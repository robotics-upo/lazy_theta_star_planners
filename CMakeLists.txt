cmake_minimum_required(VERSION 2.8.3)
project(theta_star)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  octomap_msgs
  octomap_ros
  moveit_msgs
  pcl_conversions
  pcl_ros
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES theta_star
  CATKIN_DEPENDS octomap_msgs octomap_ros pcl_conversions pcl_ros roscpp
  DEPENDS octomap_msgs octomap_ros
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_library(theta_star src/ThetaStar.cpp)

add_executable(theta_star_example src/theta_star_example.cpp)
target_link_libraries(theta_star_example ${catkin_LIBRARIES} theta_star ${octomap_ros_LIBRARIES} ${octomap_msgs_LIBRARIES})
add_dependencies(theta_star_example ${catkin_EXPORTED_TARGETS})
