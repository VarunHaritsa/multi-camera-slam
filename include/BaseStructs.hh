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

} // namespace slam
