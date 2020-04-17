#include "slam/orbExtractor.h"
#include "slam/keyframe.h"

using namespace std;
using namespace cv::cuda;

namespace customSlam{
  getORBFeatures::getORBFeatures(){
    orbGpu = ORB::create(1000);
    keyDB = new keyFrameDatabase;
    matcher = DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
  }

  void getORBFeatures::extractFeatures(GpuMat& img1,  GpuMat& img2, GpuMat& img3){
    orbGpu->detect(img1, key1);
    orbGpu->compute(img1, key1, desc1);

    orbGpu->detect(img2, key2);
    orbGpu->compute(img2, key2, desc2);

    orbGpu->detect(img3, key3);
    orbGpu->compute(img3, key3, desc3);

    // Check if the database is empty
    if(keyDB->kfDatabaseMap.empty()) createInitialMap();

    /* Only for feature display*/
    /* Comment out for higher speed computations*/
    img1.download(in1);
    img2.download(in2);
    img3.download(in3);

    cv::drawKeypoints(in1, key1, out1, cv::Scalar(0,255,0));
    cv::drawKeypoints(in2, key2, out2, cv::Scalar(0,255,0));
    cv::drawKeypoints(in3, key3, out3, cv::Scalar(0,255,0));

    cv::imshow("View1", out1);
    cv::waitKey(1);

    cv::imshow("View2", out2);
    cv::waitKey(1);

    cv::imshow("View3", out3);
    cv::waitKey(1);
    /*End of display code snippet*/

    // Debug code for checking contents of the keyframe database
    /*for(unordered_map<int, keyFrame*>::iterator it=keyDB->kfMap.begin(); it!=keyDB->kfMap.end(); ++it){
      cout << it->first << " " << it->second->frameID << endl;
    }*/

    // Match the features and decide whether it's a keyframe or Not
    if(isKeyFrame()) return;
  }

  // Determines the given multi-frame qualifies as a new keyframe
  bool getORBFeatures::isKeyFrame(){
    // Knn feature matching and other conditions (minFrames = fps/3, maxFrames = 2*fps/3)
  }

  // Creates the initial map of the envirnoment and starts filling the keyframe databases
  void getORBFeatures::createInitialMap(){
    cout << "Creating an initial map" << endl;
    geometry_msgs::Vector3 initPose;
    initPose.x = 0;
    initPose.y = 0;
    initPose.z = 0;
    keyDB->kfDatabaseMap.insert(pair<int, geometry_msgs::Vector3>(1,initPose));
    newMultiFrame = keyFrame(1,1,{key1, key2, key3}, {desc1, desc2, desc3});
    keyFrame *ptr = &newMultiFrame;
    keyDB->kfMap.insert(pair<int, keyFrame*>(1, ptr));
  }

  void getORBFeatures::matchFeatureInDB(){}
}
