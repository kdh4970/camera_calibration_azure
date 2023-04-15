#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageSub 

{

public:
ImageSub(int pargc, char **pArgv, ros::NodeHandle* nh, int camnum, std::string topic);
void ImageCallbacks(const sensor_msgs::ImageConstPtr &msg);
void CameraInfoCallbacks(const sensor_msgs::CameraInfoConstPtr &msg);
float fx, fy, cx, cy;

private:
image_transport::Subscriber image_sub;
image_transport::Subscriber depth_sub;
ros::NodeHandle* m_nh;
int m_camnum;
int depthRow;
int depthCol;
std::string cameraTopic;
cv::Mat depth;
cv::Mat rgb;
char** m_pArgv;
int m_argc;
bool justonce;


};