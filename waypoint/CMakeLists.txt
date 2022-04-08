cmake_minimum_required(VERSION 3.0.2)
project(waypoint_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS interactive_markers roscpp visualization_msgs tf)
catkin_package(
  CATKIN_DEPENDS interactive_markers roscpp visualization_msgs tf
)

###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(wayp_edit src/wayp_edit.cpp)
target_link_libraries(wayp_edit
   ${catkin_LIBRARIES}
)
add_executable(waypoint_pub src/wayp_pub.cpp)
target_link_libraries(waypoint_pub
   ${catkin_LIBRARIES}
)
add_executable(wayp_command src/wayp_command.cpp)
target_link_libraries(wayp_command
   ${catkin_LIBRARIES}
)
add_executable(wayp_auto src/wayp_auto.cpp)
target_link_libraries(wayp_auto
   ${catkin_LIBRARIES}
)
