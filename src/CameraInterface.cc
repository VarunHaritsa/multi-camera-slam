#include "CameraInterface.hh"

#include <cv_bridge/cv_bridge.h>
#include <iostream>

namespace
{

    constexpr auto QUEUE_SIZE = 1;
    static auto SEQUENCE_ID = static_cast<size_t>(0);

} // namespace

namespace slam
{

    CameraInterface::CameraInterface() 
    {
    }

    CameraInterface::~CameraInterface()
    {
        if (m_CameraPtr != nullptr)
            m_CameraPtr->release();
        if (m_ImagePublisher != nullptr)
            m_ImagePublisher->shutdown();
    }

    bool CameraInterface::Initialize(const CameraModeEnum &mode, const ros::NodeHandle &nh)
    {
        image_transport::ImageTransport it(nh);
        switch (mode)
        {
        case CameraModeEnum::LIVE_CAMERA:
        {
            m_ImagePublisher = std::make_unique<image_transport::Publisher>(
                it.advertise("live_camera/images", QUEUE_SIZE));
            m_CameraPtr = std::make_unique<cv::VideoCapture>();
            if (!m_CameraPtr->open(0))
            {
                ROS_ERROR_STREAM("Cannot open camera");
                return false;
            }

            ROS_INFO_STREAM("Camera interface setup and ready to stream");
            return true;
        }

        default:
        {
            ROS_ERROR_STREAM("Failed to initialize camera interface, reboot the program !");
            return false;
        }
        }
    }

    bool CameraInterface::Run()
    {
        cv::Mat image;
        if (!m_CameraPtr->read(image))
            return false;

        try
        {
            m_Header.seq = static_cast<uint32_t>(SEQUENCE_ID++);
            m_Header.stamp = ros::Time::now();
            m_Header.frame_id = "camera";

            auto imageMsg = cv_bridge::CvImage(m_Header, "bgr8", std::move(image)).toImageMsg();
            m_ImagePublisher->publish(std::move(imageMsg));
        }
        catch (const cv_bridge::Exception &e)
        {
            ROS_ERROR("Cv bridge conversion failed: %s", e.what());
            return false;
        }

        return true;
    }

} // namespace slam
