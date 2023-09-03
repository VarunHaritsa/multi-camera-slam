#include "FeatureExtractor.hh"

#include <opencv2/core/eigen.hpp>

namespace slam
{

    VisualFeatureExtractor::VisualFeatureExtractor() : m_CvOrbPtr(cv::ORB::create())
    {
    }

    VisualFeatureExtractor::~VisualFeatureExtractor() = default;

    bool VisualFeatureExtractor::ExtractFeatures(cv::InputArray image)
    {
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;

        m_Features.clear();
        try
        {
            m_CvOrbPtr->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
            for (size_t idx = 0; idx < keypoints.size(); idx++)
            {
                m_Features.emplace_back(VisualFeature{
                    .keypoint = Eigen::Vector2d(static_cast<double>(keypoints.at(idx).pt.x),
                                                static_cast<double>(keypoints.at(idx).pt.y)),
                    .descriptor = ToEigenMat(descriptors.row(idx))});
            }
        }
        catch (const cv::Exception &e)
        {
            ROS_ERROR("Feature extraction failed: %s", e.what());
            return false;
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Feature extraction failed: %s", e.what());
            return false;
        }

        RenderFeatures(image.getMat(), std::move(keypoints));
        return true;
    }

    Eigen::MatrixXd VisualFeatureExtractor::ToEigenMat(const cv::Mat &matrix) const
    {
        Eigen::MatrixXf result;
        cv::cv2eigen(matrix, result);
        return result.cast<double>();
    }

    void VisualFeatureExtractor::RenderFeatures(const cv::Mat &image, const std::vector<cv::KeyPoint> &keypoints)
    {
        cv::Mat result;
        cv::drawKeypoints(image, std::move(keypoints), result, cv::Scalar(0.0, 255.0, 0.0));
        result.convertTo(result, CV_8UC3);
        m_ImageToRender = std::make_optional<cv::Mat>(std::move(result));
    }

    const std::vector<VisualFeature> &VisualFeatureExtractor::Features() const
    {
        return m_Features;
    }

    const std::optional<cv::Mat> &VisualFeatureExtractor::ImageWithKeypoints() const
    {
        return m_ImageToRender;
    }

} // namespace slam
