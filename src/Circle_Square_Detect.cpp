#include "ros/ros.h"
#include "stdlib.h"
#include "math.h"
#include "time.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "autobot/Num.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;

ros::Publisher publish_data;
geometry_msgs::Twist data_msg;


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
void image_recive(const sensor_msgs::Image::ConstPtr& msg){

    cv_bridge::CvImagePtr cv_image = convert(msg);
    bool process_value = process(cv_image->image);

    if(process_value){
        data_msg.linear.x = 5;
        data_msg.angular.z = 0;
        publish_data.publish(data_msg);
    }
    else{
        data_msg.linear.x = 0;
        data_msg.angular.z = 0;
        publish_data.publish(data_msg);
        publish_data.publish(data_msg);
    }
}



bool process(Mat frame){
    Mat gray;
    resize(frame, frame, Size(640,480), INTER_LINEAR);
    cvtColor(frame, gray, COLOR_RGB2GRAY);
    
    Mat lower_red_hue_range;
    Mat upper_red_hue_range;
    inRange(frame,Scalar(0,100,100),Scalar(10,255,255),lower_red_hue_range);
    inRange(frame,Scalar(160,100,100),Scalar(179,255,255),upper_red_hue_range);

    Mat red_hue_Image;
    addWeighted(lower_red_hue_range,1.0,upper_red_hue_range,1.0,0.0,red_hue_Image);

    GaussianBlur(red_hue_Image,red_hue_Image,Size(9,9),2,2);

    vector<Vec3f> circles;
    HoughCircles(red_hue_Image,circles,CV_HOUGH_GRADIENT,1,red_hue_Image.rows/8,100,20,0,0);

    if(circles.size() == 0){
        return false;
    }
    else{
        return true;
    }
    
}
void contro_sig_recive(const autobot::Num msg){

}

void Init_system(void){
    printf("System Initialize... \n");
}
int main(int argc, char **argv){
    ros::init(argc,argv,"Circle_Square_Detection");
    ros::NodeHandle nodehandle;
    publish_data = nodehandle.advertise<geometry_msgs::Twist>("cmd_vel",1000);

    ros::NodeHandle get_image;
    ros::Subscriber topic_sub = get_image.subscribe("/camera/rgb/image_raw",1000,image_recive);

    ros::NodeHandle contro_sig;
    ros::Subscriber another_topic_sub = contro_sig.subscribe("/controller_topic",1000,contro_sig_recive);

    Init_system();
    ros::spin();

    return 0;
}
