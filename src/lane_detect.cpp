#include "ros/ros.h"
#include "stdlib.h"
#include "math.h"
#include "time.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "autobot/Num.h"
#include "std_msgs/String.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// using namespace cv;
// using namespace std;

void image_receive(const sensor_msgs::Image::ConstPtr& msg)
{
  try 
  {
    cv::imshow("view",cv_bridge::toCvShare(msg,"bgr8")->image);
    cv::waitKey(30);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_detect");
    ros::NodeHandle n;
    cv::namedWindow("view");

    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("/camera/image", 1000, image_receive);

    ros::spin();
    cv::destroyWindow("view");

    return 0;
}