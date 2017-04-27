#include "iostream"
#include "fstream"

#include "vrpnNode.cpp"
#include "CameraTrackable.cpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pgrcamera_driver/TakePicAction.h>
#include <boost/bind.hpp>

#include "FlyCapture2.h"

#define SERIAL_LEFT 13200942
#define SERIAL_RIGHT 13200944

using namespace FlyCapture2;
using namespace cv;

void convert_image(const sensor_msgs::ImageConstPtr& msg,Mat* image){
  sensor_msgs::Image imag=*msg;
  //std::cout << "Image time is: " << imag.header.stamp << std::endl;
  cv_bridge::CvImagePtr cv_ptr;
  //printf("Got image\n");
  ros::spinOnce();
    try
      {
        cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");//sensor_msgs::image_encodings::BGR8);                                                                                                                  
       }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    cv::Mat image_out=cv_ptr->image;
    image_out.copyTo(*image);//left_images.push(image_out);                                                                         
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "inputScanner");
  ros::NodeHandle n;
  cv::namedWindow("left",CV_WINDOW_NORMAL);
  cv::namedWindow("right",CV_WINDOW_NORMAL);

  ROS_INFO("ENtered The LOop");

  actionlib::SimpleActionClient<pgrcamera_driver::TakePicAction> ac("take_pic", true); //First
  
  cv::Mat image_left,image_right;

  image_transport::ImageTransport it(n);
  image_transport::Subscriber pic_left=it.subscribe("/stereo/left/image_rect_color",1,boost::bind(convert_image,_1,&image_left));
  image_transport::Subscriber pic_right=it.subscribe("/stereo/right/image_rect_color",1,boost::bind(convert_image,_1,&image_right));

  pgrcamera_driver::TakePicGoal goal;
  goal.acquire=true;
  
    bool updated=false;
    bool started=false;
  ros::Time now=ros::Time::now();
  ros::Time last=ros::Time::now();
  char exit=-1;


int count = 0;
while (ros::ok() && count!=1)
{
 ros::spinOnce();
    if (!updated){
    ac.waitForServer();
    ac.sendGoal(goal);
    }
    
    if (!image_left.empty() && !image_right.empty()){
	updated=true;
        ROS_INFO("loop pic stored");
	imshow("left",image_left);
	imshow("right",image_right);
        exit=waitKey(100);
     	now=ros::Time::now();

 	ac.sendGoal(goal);
	ros::spinOnce();
	if (!started){
	  if((now-last).toSec()>10){
	    started=true;
	  }
	}

    ros::Duration(5).sleep();
    char filename_left[512];
	sprintf( filename_left, "/home/dukerama/catkin_ws/src/pgr_calibration/include/calib_images/left_cap2.bmp");
	  
    char filename_right[512];
	sprintf( filename_right, "/home/dukerama/catkin_ws/src/pgr_calibration/include/calib_images/right_cap2.bmp");

	  imwrite(filename_left,image_left);
	  imwrite(filename_right,image_right);

count ++;

}
}
  printf( "\nFinished grabbing images\n" );
  return 0;
}




