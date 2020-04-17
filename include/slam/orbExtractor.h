#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

// General C++ headers
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <algorithm>

// OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "slam/keyframeDatabase.h"
#include "slam/keyframe.h"

using namespace std;
using namespace cv::cuda;

namespace customSlam{
  class keyFrameDatabase;
  class getORBFeatures{
  public:
    cv::Ptr<ORB> orbGpu;
    cv::Ptr<DescriptorMatcher> matcher;
    GpuMat desc1, desc2, desc3;
    vector<cv::KeyPoint> key1, key2, key3;
    cv::Mat out1, out2, out3, in1, in2, in3;

    getORBFeatures();
    void extractFeatures(GpuMat img1,  GpuMat img2, GpuMat img3);
    void matchFeatureInDB();
    void createInitialMap();
    bool isKeyFrame();

  private:
    keyFrameDatabase* keyDB;
    keyFrame newMultiFrame;
  };
}
#endif
