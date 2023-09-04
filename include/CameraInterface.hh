#pragma once

#include <memory>

#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

namespace slam
{

    enum CameraModeEnum
    {
        LIVE_CAMERA = 0,
        IMAGE_STREAM = 1
    };

    class CameraInterface
    {
    public:
        /**
         * @brief Default constructor
         */
        CameraInterface();
        /**
         * @brief Default destructor
         */
        ~CameraInterface();
        /**
         * @brief Initilizes the video capture object to handle the camera and publisher
         * @param [in] mode The mode of operation for the camera
         * @param [in] nh The ROS node handle for the visual SLAM node
         * @returns Flag indicating the success / failure of initialization
         */
        bool Initialize(const CameraModeEnum &mode, const ros::NodeHandle &nh);
        /**
         * @brief Runs one cycle of the interface, reads an image from the camera and publishes as a serializable
         *        ROS sensor message
         */
        bool Run();

    private:
        /// @brief Pointer for the camera object in Opencv
        std::unique_ptr<cv::VideoCapture> m_CameraPtr{nullptr};
        /// @brief Pointer to the image transport publisher
        std::unique_ptr<image_transport::Publisher> m_ImagePublisher{nullptr};

        /// @brief The header to be applied to the serializable ROS message
        std_msgs::Header m_Header;
    };

} // namespace slam
