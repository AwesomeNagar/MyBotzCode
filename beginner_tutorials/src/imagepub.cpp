#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
using namespace std;
using namespace cv;
using namespace cv::ximgproc;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imagepub");

  //FRONT IS THE TOPIC AND NODE FOR THE FRONTCAM
  //DOWN IS TOPIC AND NODE FOR THE DOWNCAM

  ros::NodeHandle front;
  ros::NodeHandle down;
  ros::Publisher front_image_publisher = front.advertise<sensor_msgs::Image>("frontimage", 1000);
  ros::Publisher down_image_publisher = down.advertise<sensor_msgs::Image>("downimage", 1000);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    Mat frontimage;
    Mat downimage;

    //REPLACE WITH SHARED MEMORY PATH FOR FRONTCAM
    Mat frontfimage = imread("/home/suhas/Downloads/9fc2.png",IMREAD_COLOR);

    //REPLACE WITH SHARED MEMORY PATH FOR DOWNCAM
    Mat downfimage = imread("/home/suhas/Downloads/SnowieNagar.jpg",IMREAD_COLOR);

    //CHANGE IMADE FROM BGR FORMAT TO RGB FORMAT
    cvtColor(frontfimage,frontimage,CV_BGR2RGB);
    cvtColor(downfimage,downimage,CV_BGR2RGB);
    cv_bridge::CvImage out_msg_front;
    cv_bridge::CvImage out_msg_down;

    //ENCODE THE IMAGE INTO RGB8 FORMAT
    out_msg_front.encoding = sensor_msgs::image_encodings::RGB8;
    out_msg_down.encoding = sensor_msgs::image_encodings::RGB8;

    //SAVE IMAGE INTO MESSAGE THAT WILL BE PUBLISHED
    out_msg_front.image = frontimage;
    out_msg_down.image = downimage;

    //PUBLISH THE IMAGE INTO TOPICS

    front_image_publisher.publish(out_msg_front.toImageMsg());
    down_image_publisher.publish(out_msg_down.toImageMsg());


    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}

