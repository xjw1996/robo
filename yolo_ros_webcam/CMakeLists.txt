cmake_minimum_required(VERSION 3.0.2)
project(zed_yolo_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  darknet_ros
  cv_bridge
  image_transport
  sensor_msgs
  actionlib
  zed_yolo_msg
  geometry_msgs
  tf
)
find_package(ZED 3)
find_package(OpenCV)
find_package(CUDA)
find_package(Boost REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES zed_ros
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${ZED_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)
link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(/usr/local/cuda/lib64)
if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
    message(STATUS "LINK_SHARED_ZED on")
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
    message(STATUS "LINK_SHARED_ZED off")
endif()

add_executable(zed_yolo_ros
  src/zed_yolo_ros.cpp
)
target_link_libraries(zed_yolo_ros
  ${catkin_LIBRARIES}
  ${ZED_LIBS}
  ${SPECIAL_OS_LIBS}
)
add_executable(zed_action
  src/zed_action.cpp
)
target_link_libraries(zed_action
  ${catkin_LIBRARIES}
  ${ZED_LIBS}
  ${SPECIAL_OS_LIBS}
)
add_executable(zed_server
  src/zed_server.cpp
)
target_link_libraries(zed_server
  ${catkin_LIBRARIES}
  ${ZED_LIBS}
  ${SPECIAL_OS_LIBS}
)
message(STATUS "CUDA Libararies: ${CUDA_LIBRARIES}")
message(STATUS "CUDA c Libararies: ${CUDA_CUDA_LIBRARY}")
message(STATUS "CUDA cudart Libararies: ${CUDA_CUDART_LIBRARY}")

