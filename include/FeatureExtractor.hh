#pragma once

#include <vector>
#include <optional>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/flann/flann.hpp>

#include "BaseStructs.hh"

namespace slam
{

    class VisualFeatureExtractor
    {
    public:
        /**
         * @brief Default constructor for feature extractor
         */
        VisualFeatureExtractor();
        /**
         * @brief Default desctructor
         */
        ~VisualFeatureExtractor();
        /**
         * @brief Method to extract sift features from an input floating point image
         * @param [in] image The input image
         * @returns Boolean indicating the success / failure of the step
         */
        bool ExtractFeatures(cv::InputArray image);

        /// @brief Get access to the internal feature set
        const std::vector<VisualFeature> &Features() const;
        /// @brief Get access to the image to be rendered with the keypoints 
        const std::optional<cv::Mat> &ImageWithKeypoints() const;

    private:
        /**
         * @brief Conversion method to go from opencv matrix to eigen matrix
         * @param [in] matrix The opencv 32-bit floating point precise matrix
         * @returns Eigen matrix with same size of type double
         */
        Eigen::MatrixXd ToEigenMat(const cv::Mat &matrix) const;
        /**
         * @brief Render the keypoints on the corresponding image update the image to be rendered
         * @param [in] image The 8-bit unsigned int image
         * @param [in] keypoints Vector of keypoints computed in the current run
         */
        void RenderFeatures(const cv::Mat &image, const std::vector<cv::KeyPoint> &keypoints);

    private:
        /// @brief Pointer to the OpenCV ORB feature extractor
        cv::Ptr<cv::ORB> m_CvOrbPtr{nullptr};
        /// @brief Vector of features from the latest run
        std::vector<VisualFeature> m_Features;

        /// @brief Debug image is set to a value only if flag is set
        std::optional<cv::Mat> m_ImageToRender;
    };

} // namespace slam
