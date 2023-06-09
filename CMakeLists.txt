cmake_minimum_required(VERSION 2.8.3)
project(rs_wrapper)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_filters
  sensor_msgs
  cv_bridge
  rosbag
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES img_pub
   CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

# CXX flags 
set(CMAKE_CXX_FLAGS "-ggdb -g -O3 -w -std=c++11 -fpermissive") # -fPIC -fpermissive

set(LIBREAL_SENSE_DIR "/home/gccrobot/3rdlibrary/librealsense")

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ./src
  "/usr/include/eigen3"
  ${LIBREAL_SENSE_DIR}/include
)
####### OpenCV #######
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS} ${LIBREAL_SENSE_DIR}/build)

###########
## Build ##
###########

####### img_pub library #######
add_executable(rs_wrapper rs_wrapper.cpp)
target_link_libraries(rs_wrapper ${catkin_LIBRARIES} ${OpenCV_LIBS} realsense2)
