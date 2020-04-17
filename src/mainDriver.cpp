// General C++ headers
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

// OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/cudaimgproc.hpp>

// Custom headers
//#include <multi_cam_driver/camSubscriber.h>
#include <slam/orbExtractor.h>

using namespace std;
using namespace cv::cuda;
using namespace customSlam;

int main(int argc, char** argv){
  ros::init(argc, argv, "slam");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	// Camera driver code
	cv::VideoCapture cam0, cam1, cam2;
	//ros::Rate rate(10);

	cam0.open("/dev/video0"+cv::CAP_ANY);
	cam1.open("/dev/video1"+cv::CAP_ANY);
	cam2.open("/dev/video2"+cv::CAP_ANY);
	if(!cam0.isOpened() || !cam1.isOpened() || !cam2.isOpened()){
		ROS_ERROR("Cameras not opened");
		return -1;
	}

  getORBFeatures *orb = new getORBFeatures;
	cv::Mat frame0, frame1, frame2;

  while(ros::ok()){
		ros::spinOnce();
		cam0 >> frame0;
		cam1 >> frame1;
		cam2 >> frame2;

		cv::cvtColor(frame0, frame0, cv::COLOR_RGB2GRAY);
		cv::cvtColor(frame1, frame1, cv::COLOR_RGB2GRAY);
		cv::cvtColor(frame2, frame2, cv::COLOR_RGB2GRAY);

		try{
			GpuMat img1(frame0);
			GpuMat img2(frame1);
			GpuMat img3(frame2);
			orb->extractFeatures(img1, img2, img3);
		}
		catch(cv::Exception& e){
			cout << "hold on" << endl;
		}
  }
	cv::destroyAllWindows();
  return 0;
}
