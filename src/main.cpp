#include "FeatureExtractor.hh"
#include "FeatureTracker.hh"

#include <optional>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace
{

    constexpr auto DEVICE_INDEX = 0;
    constexpr auto RENDER_IMAGE = true;

    void Render(const std::optional<cv::Mat> &imageWithKeypoints,
                const std::optional<cv::Mat> &imageWithMatches)
    {
        if (imageWithMatches.has_value())
            cv::imshow("ImageWithMatches", imageWithMatches.value());

        else if (imageWithKeypoints.has_value())
            cv::imshow("ImageWithFeatures", imageWithKeypoints.value());

        // Wait for 1 ms regardless
        cv::waitKey(1);
    }

} // namespace

int main(int argc, char **argv)
{
    // Initialize ros node
    ros::init(argc, argv, "VisualSlamNode");
    ros::NodeHandle nh;
    cv::Mat image;
    std::vector<slam::VisualFeature> reference;
    ROS_INFO_STREAM("Successfully initialized visual slam node");

    auto intrinsics = Eigen::Matrix3d::Identity().eval();
    auto camera = cv::VideoCapture();
    auto orb = slam::VisualFeatureExtractor();
    auto track = slam::VisualFeatureTracker();

    if (!camera.open(DEVICE_INDEX))
    {
        ROS_ERROR_STREAM("Failed to open video camera");
        return EXIT_FAILURE;
    }

    ros::Rate loopRate(10);
    while (ros::ok())
    {
        // Actual pipeline exectution
        camera.read(image);
        auto feature_extraction_status = orb.ExtractFeatures(image);
        if (!reference.empty())
        {
            auto feature_tracking_status = track.MatchFeatures(slam::FeatureTrackerInput{
                                                                   .trainFeatures = reference,
                                                                   .queryFeatures = orb.Features(),
                                                                   .cameraIntrinsics = intrinsics},
                                                               std::make_optional(std::cref<cv::Mat>(image)));

            // Rendering if the flag is enabled
            if (RENDER_IMAGE)
                Render(orb.ImageWithKeypoints(), track.ImageWithMatches());
        }

        // ROS loop spinning
        reference = orb.Features();
        ros::spinOnce();
        loopRate.sleep();
    }

    camera.release();
    cv::destroyAllWindows();
    return EXIT_SUCCESS;
}
