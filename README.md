# EcoPRT_custom_SLAM
Customized visual inertial SLAM solution for the EcoPRT reseach project (2-person autonomous vehicle). This is still under development and written completely in C++11 (so far). This uses the OpenCV-CUDA libraries for faster computations. For more information on CUDA support for OpenCV modules, please visit *https://opencv.org/platforms/cuda/*

# System requirements and dependencies
Ubuntu 16.04
OpenCV-3.4.0
CUDA-10.0
ROS-Kinetic

# Installation
CUDA-10.0 installation from scratch is given in the link --> http://www.rignitc.com/2018/12/29/install-cuda-10-with-ubuntu-16-04/

The first package to install is a CUDA-10.0 compatible version of OpenCV. The installation instructions are given in the link --> *https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html*. To build with CUDA just add the commands replace the cmake command from the link to 
*cmake -D CMAKE_BUILD_TYPE=Release -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -D CMAKE_INSTALL_PREFIX=/usr/local -D ENABLE_PRECOMPILED_HEADERS=OFF -D WITH_OPENCL=OFF -D WITH_OPENMP=OFF -D WITH_FFMPEG=ON -D WITH_CUDA=ON -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-10.0 -D CUDA_ARCH_BIN=5.3 -D CUDA_ARCH_PTX="" -D OPENCV_TEST_DATA_PATH=../opencv_extra/testdata ../opencv*

# Running the SLAM 
Clone the git repository (git clone *repository URL*)
Enter you catkin workspace (cd *catkin_workspace_name*)
Compile the code (*catkin_make* or *catkin build* depending on how you've configured ROS)

To run the slam node, start a ROS master using roscore and run the executable we get after compilation using rosrun *package name* *executable name*
