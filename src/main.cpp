#include "FeatureExtractor.hh"

#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace
{

    constexpr auto DEVICE_INDEX = 0;

    void Render(const std::optional<cv::Mat> &imageWithKeypoints) 
    {
        if (imageWithKeypoints.has_value())
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
    ROS_INFO_STREAM("Successfully initialized visual slam node");
    
    auto camera = cv::VideoCapture();
    auto orb = slam::VisualFeatureExtractor();
    if (!camera.open(DEVICE_INDEX))
    {
        ROS_ERROR_STREAM("Failed to open video camera");
        return EXIT_FAILURE;
    }

    ros::Rate loopRate(10);
    while (ros::ok())
    {
        camera.read(image);   
        if (!orb.ExtractFeatures(image))
        {
            ROS_ERROR_ONCE("Couldn't extract features from the image");
            camera.release();
            return EXIT_FAILURE;
        }

        Render(orb.ImageWithKeypoints());
        ros::spinOnce();
        loopRate.sleep();
    }

    camera.release();
    cv::destroyAllWindows();
    return EXIT_SUCCESS;
}
