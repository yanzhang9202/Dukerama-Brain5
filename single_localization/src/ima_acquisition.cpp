// Created by Yan Zhang, 06/01/2015

#include "iostream"
#include "fstream"

#include <Eigen/Dense>
#include <geometry_msgs/TransformStamped.h>

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

#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include "FlyCapture2.h"

#define SERIAL_LEFT 13200942
#define SERIAL_RIGHT 13200944

using namespace FlyCapture2;
using namespace cv;

void recordHandPoseCallback(const geometry_msgs::TransformStamped msg);

Eigen::VectorXf q_h(4), ohand_world(3);

void waitFor(unsigned int secs){
	unsigned int retTime;
	retTime = time(0) + secs;
	while (time(0) < retTime);
}
void waitForEnter(){
	string Enter;
	std::cin >> Enter;
}
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
  cv::namedWindow("right",CV_WINDOW_NORMAL);
  cv::namedWindow("left",CV_WINDOW_NORMAL);

  std::ofstream outfile_hand("/home/dukerama/hydro_catkin/src/single_localization/include/positions/Positions.txt");

  ros::Subscriber cam_pose_sub = n.subscribe("/Brain5Camera/pose", 1, recordHandPoseCallback);
  
  std::ofstream outfile_left("/home/dukerama/hydro_catkin/src/single_localization/include/detector/cords_left.txt");
  std::ofstream outfile_right("/home/dukerama/hydro_catkin/src/single_localization/include/detector/cords_right.txt");

  actionlib::SimpleActionClient<pgrcamera_driver::TakePicAction> ac("Brain5/take_pic", true); //First argument in ac constructor is the server to connect to

  cv::Mat image_left,image_right;

  std::string myself="Brain5Camera";

  image_transport::ImageTransport it(n);
  image_transport::Subscriber pic_left=it.subscribe("/Brain5/left/image_rect_color",1,boost::bind(convert_image,_1,&image_left));
  image_transport::Subscriber pic_right=it.subscribe("/Brain5/right/image_rect_color",1,boost::bind(convert_image,_1,&image_right));

  pgrcamera_driver::TakePicGoal goal;
  goal.acquire=true;
  
  ros::Rate loop_rate(60);

  bool updated=false;
  bool started=false;
  int count=601;
  char exit=-1;

  int num_ima=0;

  ros::Time now=ros::Time::now();
  ros::Time last=ros::Time::now();

  int update_cnt = 1;
  
	
  std::cout<<"\n Initiating Camera... \n";

//  int edgeThresh = 1;
//  int const lowThreshold = 30;
//  int const max_lowThreshold = 100;
//  int ratio = 3;
//  int kernel_size = 3;

  //ping camera driver and wait for it to start
  while (ros::ok() && !updated)
  {
	ac.waitForServer();
	ac.sendGoal(goal);

	if (!image_left.empty() && !image_right.empty())
	{
		imshow("right",image_right);
		imshow("left",image_left);
		cv::waitKey(30);
		updated = true;
	}
	
	ros::spinOnce();
	loop_rate.sleep();
  }	

  while (ros::ok() && num_ima<1)
    {
	//Set up station
	std::cout<<"\n Set up your station: ";
	std::cout<<num_ima;
	waitForEnter();
	
	//take picture
	ac.sendGoal(goal);
	ac.waitForResult();
	ac.sendGoal(goal);
	ac.waitForResult();
	ac.sendGoal(goal);
	ac.waitForResult();
	ros::spinOnce();
	loop_rate.sleep();

	//show image to image window
	imshow("right",image_right);
	cv::waitKey(30);
	imshow("left",image_left);
	cv::waitKey(30);
	
	//write image to file
	char filename_left[512];
	sprintf( filename_left, "/home/dukerama/hydro_catkin/src/single_localization/include/calib_images/left%d.bmp", count );
	imwrite(filename_left,image_left);

	char filename_right[512];
	sprintf( filename_right, "/home/dukerama/hydro_catkin/src/single_localization/include/calib_images/right%d.bmp", count );
	imwrite(filename_right,image_right);

////************************* Circle detector Started *****************************//
////**************** Left Camera ******************//
//	cv::Mat detected_edges;
//	cv::Mat image_gray;

////	dst.create(image_left.size(), image_left.type());
//	cv::cvtColor(image_left, image_gray, CV_BGR2GRAY);

////	/// Reduce noise with a kernel 3x3
////	//cv::blur( image_left_gray, detected_edges, Size(3,3) );	
//        cv::GaussianBlur(image_gray,detected_edges,cv::Size(5,5),0,0);

////	/// Canny detector
////	cv::Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
//	
//	std::vector<cv::Vec3f> circles;
//        cv::HoughCircles( detected_edges, circles, CV_HOUGH_GRADIENT, 1, detected_edges.cols/100, 120, 18, 0, 20);

//	for (size_t i = 0; i < circles.size(); i++ )
//		{
//			std::cout <<"detecting " << std::endl;
//			cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//			int radius = cvRound(circles[i][2]);
//			circle( image_left, center, 3, Scalar(0, 255, 0), -1, 8, 0 );
//			circle( image_left, center, radius, Scalar(0, 0, 255), 3, 8, 0 );
//			outfile_left << circles[i][0] << " ";
//			outfile_left << circles[i][1] << " ";
//			outfile_left << circles[i][2] << " ";
//		}
//	outfile_left << "\n";

//	cv::namedWindow("Left Hough Display", CV_WINDOW_NORMAL);
//	imshow("Left Hough Display", image_left);	
//	cv::waitKey(30);

//	char filename_detector_left[512];
//	sprintf( filename_detector_left, "/home/dukerama/hydro_catkin/src/single_localization/include/detector/left%d.bmp", count );
//	imwrite(filename_detector_left,image_left);	

////******************* Right Camera *********************//

//	cv::cvtColor(image_right, image_gray, CV_BGR2GRAY);

////	/// Reduce noise with a kernel 3x3
////	//cv::blur( image_left_gray, detected_edges, Size(3,3) );	
//        cv::GaussianBlur(image_gray,detected_edges,cv::Size(5,5),0,0);

////	/// Canny detector
////	cv::Canny( detected_edges, detected_edges, 60, 120, 3 );
//	
//        cv::HoughCircles( detected_edges, circles, CV_HOUGH_GRADIENT, 1, detected_edges.cols/100, 120, 18, 0, 20);

//	for (size_t i = 0; i < circles.size(); i++ )
//		{
//			std::cout <<"detecting " << std::endl;
//			cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//			int radius = cvRound(circles[i][2]);
//			circle( image_right, center, 3, Scalar(0, 255, 0), -1, 8, 0 );
//			circle( image_right, center, radius, Scalar(0, 0, 255), 3, 8, 0 );
//			outfile_right << circles[i][0] << " ";
//			outfile_right << circles[i][1] << " ";
//			outfile_right << circles[i][2] << " ";
//		}
//	outfile_right << "\n";

//	cv::namedWindow("Right Hough Display", CV_WINDOW_NORMAL);
//	imshow("Right Hough Display", image_right);	
//	cv::waitKey(30);

//	char filename_detector_right[512];
//	sprintf( filename_detector_right, "/home/dukerama/hydro_catkin/src/single_localization/include/detector/right%d.bmp", count );
//	imwrite(filename_detector_right,image_right);

        for (int i=0;i<3;i++)
        {
          outfile_hand << ohand_world(i) << " ";
        }
        for (int i=0;i<4;i++)
        {
          outfile_hand << q_h(i) << " ";
        }
        outfile_hand << "\n";	

	num_ima++;
	count++;	
    }
//************************* Circle detector Ended *****************************//

  printf( "\nFinished grabbing images\n" );
  return 0;
}


void recordHandPoseCallback(const geometry_msgs::TransformStamped msg)
{
// This is hand's pose from VRPN
    q_h[0] =msg.transform.rotation.x;
    q_h[1] =msg.transform.rotation.y;
    q_h[2] =msg.transform.rotation.z;
    q_h[3] =msg.transform.rotation.w;
  
    ohand_world(0) = msg.transform.translation.x;
    ohand_world(1) = msg.transform.translation.y;
    ohand_world(2) = msg.transform.translation.z;
}

