#include "FeatureExtractor.hh"
#include "CameraInterface.hh"

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

int main(int argc, char **argv)
{
    // Initialize ros node
    ros::init(argc, argv, "VisualSlamNode");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Successfully initialized visual slam node");
    
    auto camera = slam::CameraInterface();
    auto orb = slam::VisualFeatureExtractor();
    camera.Initialize(slam::CameraModeEnum::LIVE_CAMERA, nh);

    ros::Rate loopRate(10);
    while (ros::ok())
    {
        // Publish the image onto the ros topic
        camera.Run();

        
        ros::spinOnce();
        loopRate.sleep();
    }

    return EXIT_SUCCESS;
}