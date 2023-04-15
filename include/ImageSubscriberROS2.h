#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageSubROS2 : public rclcpp::Node
{
public:
ImageSubROS2(int pargc, char **pArgv, int camnum, std::string topic);
void ImageCallbacks(const sensor_msgs::msg::Image::ConstPtr msg);
void CameraInfoCallbacks(const sensor_msgs::msg::CameraInfo::ConstPtr msg);
float fx, fy, cx, cy;
bool justonce;
std::shared_ptr<cv_bridge::CvImage> bridge_;

private:
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub;
int m_camnum;
int depthRow;
int depthCol;
std::string cameraTopic;
cv::Mat depth;
mutable cv::Mat rgb;
char** m_pArgv;
int m_argc;

};