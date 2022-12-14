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

using namespace cv;
using namespace std;

//callback when receive image from camera topic
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

//convert from ros msg to cv image
cv_bridge::CvImagePtr convert(const sensor_msgs::Image::ConstPtr& msg){
 	cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return NULL;
    }

	return cv_ptr;
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