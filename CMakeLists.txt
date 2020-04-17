cmake_minimum_required(VERSION 2.8.3)
project(slam)

SET(headers
include/slam/orbExtractor.h
include/slam/keyframeDatabase.h
include/slam/keyframe.h
)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_transport
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## To avoid pointing to the ROS-OpenCV 3.3.1-dev version
find_package(OpenCV REQUIRED
NO_MODULE
PATHS /home/ECE.ecoprt/installation/opencv-3.4.4/share/OpenCV
NO_DEFAULT_PATH)

find_package(CUDA 10.0 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${CUDA_INCLUDE_DIRS}
)

## Declare a C++ library
CUDA_ADD_LIBRARY(customLib SHARED
   src/orbExtractor.cc
	 src/keyframeDatabase.cc
	 src/keyframe.cc
	 ${headers}
)

catkin_package()

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(slam_node src/mainDriver.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(slam_node
customLib
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
${CUDA_LIBRARIES}
${CUDA_nppc_LIBRARIES}
${CUDA_nppc_LIBRARIES}
)

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()
