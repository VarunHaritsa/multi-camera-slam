#include "slam/keyframe.h"

using namespace std;
using namespace cv::cuda;

namespace customSlam{
  keyFrame::keyFrame(){}

  keyFrame::keyFrame(long unsigned int key_ID, long unsigned int frame_ID, const vector<vector<cv::KeyPoint>>& keys, const vector<GpuMat>& desc){
    keypoints = keys;
    descriptors = desc;
    keyID = key_ID;
    frameID = frame_ID;
  }
  vector<vector<cv::KeyPoint>> keyFrame::getKeyPoints(){
    return keypoints;
  }
  vector<GpuMat> keyFrame::getDescriptors(){
    return descriptors;
  }
}
