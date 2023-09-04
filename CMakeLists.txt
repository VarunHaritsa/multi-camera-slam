cmake_minimum_required(VERSION 3.16.3)
project(slam)

## Compile C++17 
add_compile_options(-std=c++17)

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
  cv_bridge
)

## To avoid pointing to the ROS-OpenCV 4.2 version
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(slamNode src/FeatureExtractor.cc src/CameraInterface.cc src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(slamNode ${OpenCV_LIBS} ${catkin_LIBRARIES} ${catkin_LIBRARY_DIRS})

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()
