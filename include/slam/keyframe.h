#ifndef KEYFRAME_H
#define KEYFRAME_H

// General C++ headers
#include <stdlib.h>
#include <vector>
#include <algorithm>

// OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafeatures2d.hpp>

using namespace std;
using namespace cv::cuda;

namespace customSlam{
  class keyFrame{
  public:
    keyFrame();
    keyFrame(long unsigned int key_ID, long unsigned int frame_ID, const vector<vector<cv::KeyPoint>>& keys, const vector<GpuMat>& desc);
    vector<vector<cv::KeyPoint>> getKeyPoints();
    vector<GpuMat> getDescriptors();
  //protected:
    vector<GpuMat> images;
    long unsigned int keyID;
    long unsigned int frameID;
    double timeStamp;

    /*Multi dimensional vector for keypoint
      descriptor storage (one vector from
      each camera)*/
    vector<vector<cv::KeyPoint>> keypoints;
    vector<GpuMat> descriptors;
  };
}

#endif
