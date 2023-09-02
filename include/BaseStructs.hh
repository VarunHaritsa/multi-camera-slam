#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>

namespace slam
{

    template <typename T, Eigen::Index rows, Eigen::Index cols>
    using Matrix = Eigen::Matrix<T, rows, cols, Eigen::RowMajor>;
    template <typename T, Eigen::Index size>
    using ColVector = Eigen::Vector<T, size>;
    template <typename T, Eigen::Index size>
    using RowVector = Eigen::RowVector<T, size>;

    // Typecasting for common matrix types
    using Matrix2d = Matrix<double, 2, 2>;
    using Matrix3d = Matrix<double, 3, 3>;
    using Matrix4d = Matrix<double, 4, 4>;
    using MatrixXd = Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
    using ProjectionMatrix = Matrix<double, 3, 4>;

    // Typecasting for common vector types
    using Vector2d = ColVector<double, 2>;
    using Vector3d = ColVector<double, 3>;
    using Vector4f = ColVector<double, 4>;
    using RowVector2d = RowVector<double, 2>;
    using RowVector3d = RowVector<double, 3>;
    using RowVector4f = RowVector<double, 4>;

    struct VisualFeature
    {
        Vector2d keypoint;
        MatrixXd descriptor;
    };

} // namespace slam
