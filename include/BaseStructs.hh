#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>

namespace slam
{
    struct VisualFeature
    {
        Eigen::Vector2d keypoint;
        Eigen::MatrixXd descriptor;
    };

    struct FeatureTrackerInput
    {
        std::vector<VisualFeature> trainFeatures;
        std::vector<VisualFeature> queryFeatures;
        Eigen::Matrix3d cameraIntrinsics;
    };

} // namespace slam
