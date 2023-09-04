#include "FeatureExtractor.hh"

#include <opencv2/core/eigen.hpp>

namespace
{

    constexpr auto RENDER_KEYPOINTS = true;

    const auto GAUSS_KERNEL_SIZE = cv::Size_<float>(5.0f, 5.0f);
    const auto COLOR = cv::Scalar(0, 255, 0);

} // namespace

namespace slam
{

    VisualFeatureExtractor::VisualFeatureExtractor() : m_CvOrbPtr(cv::ORB::create())
    {
    }

    VisualFeatureExtractor::~VisualFeatureExtractor() = default;

    bool VisualFeatureExtractor::ExtractFeatures(cv::InputArray image)
    {
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors, grayscale = image.getMat();

        // If it is a color image
        if (image.channels() > 1)
        {
            cv::cvtColor(image, grayscale, cv::COLOR_BGR2GRAY);
            auto sigma = std::max(static_cast<float>(GAUSS_KERNEL_SIZE.height) / 2.0f, 1.0f);
            cv::GaussianBlur(grayscale, grayscale, GAUSS_KERNEL_SIZE, sigma);
        }

        m_Features.clear();
        try
        {
            m_CvOrbPtr->detectAndCompute(grayscale, cv::noArray(), keypoints, descriptors);
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

        if (RENDER_KEYPOINTS)
            m_ImageToRender = RenderFeatures(std::move(grayscale), std::move(keypoints));
        
        return true;
    }

    Eigen::MatrixXd VisualFeatureExtractor::ToEigenMat(const cv::Mat &matrix) const
    {
        Eigen::MatrixXf result;
        cv::cv2eigen(matrix, result);
        return result.cast<double>();
    }

    cv::Mat VisualFeatureExtractor::RenderFeatures(const cv::Mat &image,
                                                   const std::vector<cv::KeyPoint> &keypoints) const
    {
        cv::Mat result;
        cv::drawKeypoints(image, std::move(keypoints), result, cv::Scalar(0.0, 255.0, 0.0));
        result.convertTo(result, CV_8UC3);
        return result;
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
