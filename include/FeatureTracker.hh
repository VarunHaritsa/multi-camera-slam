#pragma once

#include "BaseStructs.hh"

#include <opencv2/core/core.hpp>
#include <opencv2/flann/flann.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace slam
{

    class VisualFeatureTracker
    {
    public:
        /**
         * @brief Default Constructor
         */
        VisualFeatureTracker();
        /**
         * @brief Default desctructor
         */
        ~VisualFeatureTracker();
        /**
         * @brief Prepare the data for feature matching and update the good matches using Lowe's distance filtering
         * @param [in] input The input struct with the feature vectors of the current and keyframe images.
         * @param [in] image Optional debug image if rendering is enabled.
         * @returns Success / failure or finding good matches
         */
        bool MatchFeatures(const FeatureTrackerInput &input,
                           const std::optional<std::reference_wrapper<cv::Mat>> &image);

        /// @brief Public access method to get the good matches for the current frame
        const std::vector<cv::DMatch> &GoodMatches() const;
        /// @brief Public access method to get the image with feature matches (only if rendering is enabled)
        const std::optional<cv::Mat> &ImageWithMatches() const;

    private:
        /**
         * @brief Converts an Eigen descriptor matrix of type double to a 32-bit floating point opencv matrix
         *        of the same dimensions
         * @param [in] matrix The Eigen matrix to be converted
         * @returns 32-bit floating point opencv matrix
         */
        cv::Mat ToCvDescriptor(const Eigen::MatrixXd &matrix) const;
        /**
         * @brief Filter out bad matches using the Lowe's distance criterion
         * @param [in] matches All the matches computed by the opencv Flann based matcher
         * @param [in] intrinsics The camera intrinsic calibration matrix
         * @returns Vector of only "good" matches
         */
        std::vector<cv::DMatch> FilterMatches(const std::vector<std::vector<cv::DMatch>> &matches,
                                              const Eigen::Matrix3d &intrinsics) const;
        /**
         * @brief Draws the matches over a set of images and returns a unified opencv matrix
         * @param [in] matches Vector of "good" matches
         * @param [in] queryKps Vector of keypoints from the current feature set
         * @param [in] trainKps Vector of keypoints from the keyframe feature set
         * @returns 3-channel opencv matrix which is a unified image with matches
         */
        cv::Mat RenderImageWithMatches(const std::vector<cv::DMatch> &goodMatches,
                                       const std::vector<cv::KeyPoint> &queryKps,
                                       const std::vector<cv::KeyPoint> &trainKps) const;

    private:
        /// @brief Smart pointer to the opencv Flann based matcher
        cv::Ptr<cv::DescriptorMatcher> m_FlannMatcher{nullptr};
        /// @brief Vector of good matches filtered after the matching
        std::vector<cv::DMatch> m_GoodMatches;

        /// @brief Debug image for rendering current image
        std::optional<cv::Mat> m_CurrentImage;
        /// @brief Debug image for rendering keyframe image
        std::optional<cv::Mat> m_KeyframeImage;
        /// @brief Debug image with matches to display the result of the matching
        std::optional<cv::Mat> m_ImageWithMatches;
    };

} // namespace slam
