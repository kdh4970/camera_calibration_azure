#include "ImageSubscriberROS2.h"

using std::placeholders::_1;


ImageSubROS2::ImageSubROS2(int pargc, char **pArgv, int camnum, std::string topic)
: Node("ImageSubROS2_node"), m_argc(pargc), m_pArgv(pArgv), m_camnum(camnum),cameraTopic(topic)
{
    image_sub = this->create_subscription<sensor_msgs::msg::Image>(
    cameraTopic + "image_raw", 1, std::bind(&ImageSubROS2::ImageCallbacks, this, _1));
    caminfo_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    cameraTopic + "camera_info", 1, std::bind(&ImageSubROS2::CameraInfoCallbacks,this, _1));
    bridge_ = std::make_shared<cv_bridge::CvImage>();
    justonce = false;
}

void ImageSubROS2::ImageCallbacks(const sensor_msgs::msg::Image::ConstPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    std_msgs::msg::Header h;
    std::string path="/home/do/ros2_ws/src/camera_calibration_azure/images/";
    std::string rgbName = "RGB.png";
    RCLCPP_INFO(this->get_logger(), cameraTopic + "image_raw");
    try
    {
        bridge_->image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        cv::imwrite(path + std::to_string(m_camnum) + rgbName, bridge_->image);
        //cv::imshow("Image window", bridge_->image);
        //cv::waitKey(0);
    }
    catch(cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    

}
void ImageSubROS2::CameraInfoCallbacks(const sensor_msgs::msg::CameraInfo::ConstPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    std_msgs::msg::Header h;
    RCLCPP_INFO(this->get_logger(), cameraTopic);
    if(!justonce && cx < 0 && cy < 0)
    {
        fx = msg->k[0];
        fy = msg->k[4];
        cx = msg->k[2];
        cy = msg->k[5];
        this->justonce = true;
    }

}
