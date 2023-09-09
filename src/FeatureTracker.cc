#include "FeatureTracker.hh"

#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace
{

    constexpr auto LOWE_DISTANCE_THRESHOLD = 0.85f;
    constexpr auto RENDER_MATCHES = true;

} // namespace

namespace slam
{

    VisualFeatureTracker::VisualFeatureTracker()
        : m_FlannMatcher(cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED))
    {
    }

    VisualFeatureTracker::~VisualFeatureTracker() = default;

    bool VisualFeatureTracker::MatchFeatures(const FeatureTrackerInput &input,
                                             const std::optional<std::reference_wrapper<cv::Mat>> &image)
    {
        std::vector<cv::KeyPoint> trainKps, queryKps;
        cv::Mat trainDesc, queryDesc;

        // Unpack the feature vectors into opencv friendly data structures
        std::for_each(input.queryFeatures.begin(),
                      input.queryFeatures.end(),
                      [this, &queryKps, &queryDesc](const VisualFeature &ftr)
                      {
                          queryKps.emplace_back(cv::KeyPoint(ftr.keypoint.x(), ftr.keypoint.y(), 0.0f));
                          queryDesc.push_back(this->ToCvDescriptor(ftr.descriptor));
                      });
        std::for_each(input.trainFeatures.begin(),
                      input.trainFeatures.end(),
                      [this, &trainKps, &trainDesc](const VisualFeature &ftr)
                      {
                          trainKps.emplace_back(cv::KeyPoint(ftr.keypoint.x(), ftr.keypoint.y(), 0.0f));
                          trainDesc.push_back(this->ToCvDescriptor(ftr.descriptor));
                      });

        std::vector<std::vector<cv::DMatch>> matches;
        std::cout << queryDesc.size << "\n" << trainDesc.size << std::endl;
        m_FlannMatcher->knnMatch(queryDesc, trainDesc, matches, 2);
        m_GoodMatches = FilterMatches(std::move(matches), input.cameraIntrinsics);

        if (RENDER_MATCHES)
        {
            m_CurrentImage = std::make_optional(image.value().get());
            if (!m_KeyframeImage.has_value())
            {
                m_KeyframeImage = std::make_optional(image.value().get());
            }
            else
            {
                m_ImageWithMatches = RenderImageWithMatches(m_GoodMatches, std::move(queryKps), std::move(trainKps));
                m_KeyframeImage = m_CurrentImage;
            }
        }

        return true;
    }

    cv::Mat VisualFeatureTracker::ToCvDescriptor(const Eigen::MatrixXd &matrix) const
    {
        cv::Mat result;
        cv::eigen2cv(matrix, result);

        // Resize the result a row vector
        result = result.reshape(0, 1);
        result.convertTo(result, CV_32F);
        return result;
    }

    std::vector<cv::DMatch> VisualFeatureTracker::FilterMatches(const std::vector<std::vector<cv::DMatch>> &matches,
                                                                const Eigen::Matrix3d &intrinsics) const
    {
        std::vector<cv::DMatch> goodMatches;
        for (const auto &match : matches)
        {
            if (match.size() == 2)
            {
                auto closest = match.front();
                auto second = match.back();

                if (closest.distance < second.distance * LOWE_DISTANCE_THRESHOLD)
                    goodMatches.emplace_back(std::move(closest));
            }
        }
        return goodMatches;
    }

    cv::Mat VisualFeatureTracker::RenderImageWithMatches(const std::vector<cv::DMatch> &goodMatches,
                                                         const std::vector<cv::KeyPoint> &queryKps,
                                                         const std::vector<cv::KeyPoint> &trainKps) const
    {
        cv::Mat result;
        if (!m_CurrentImage.has_value() || !m_KeyframeImage.has_value())
            throw std::runtime_error("Trying to plot invalid images");

        try
        {
            cv::drawMatches(m_CurrentImage.value(),
                            std::move(queryKps),
                            m_KeyframeImage.value(),
                            std::move(trainKps),
                            goodMatches,
                            result,
                            cv::Scalar(0.0, 255.0, 0.0));
        }
        catch (const cv::Exception &e)
        {
            ROS_WARN("Exception caught while drawing matches: %s", e.what());
            result = m_CurrentImage.value();
        }
        result.convertTo(result, CV_8UC3);
        return result;
    }

    const std::vector<cv::DMatch> &VisualFeatureTracker::GoodMatches() const
    {
        return m_GoodMatches;
    }

    const std::optional<cv::Mat> &VisualFeatureTracker::ImageWithMatches() const
    {
        return m_ImageWithMatches;
    }

} // namespace slam
