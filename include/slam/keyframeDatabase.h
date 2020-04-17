#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <opencv2/opencv.hpp>

#include "slam/keyframe.h"

// ROS header
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

using namespace std;
using namespace cv::cuda;

namespace customSlam{
  class keyFrame;
  class keyFrameDatabase{
  public:
    keyFrameDatabase();
    void addNewKeyFrame(keyFrame* kf);
    void removeKeyFrame(keyFrame* kf);
    void clear();

    /*Map which holds the Keyframe ID which
      maps to a global position coordinates
      This can be used for visual localization
      of the robot after feature matching and
      identifying the appropriate keyframe*/
    unordered_map<int, geometry_msgs::Vector3> kfDatabaseMap;
    unordered_map<int, keyFrame*> kfMap;
  };
}

#endif
