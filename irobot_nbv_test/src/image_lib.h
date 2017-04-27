#ifndef ___IMAGE_LIB_H____
#define ___IMAGE_LIB_H____

#include <ros/ros.h>
#include <iostream>
#include <string>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pgrcamera_driver/TakePicAction.h>
#include <boost/bind.hpp>

#include "FlyCapture2.h"

class pgr_camera{	
private:
//	std::string msg;
	image_transport::Subscriber pic_left;
	image_transport::Subscriber pic_right;
  	pgrcamera_driver::TakePicGoal goal;
	actionlib::SimpleActionClient<pgrcamera_driver::TakePicAction> ac;
	ros::Rate loop_rate;
//	bool show_stereo_image;

	void convert_image(const sensor_msgs::ImageConstPtr& msg,cv::Mat* image);	

public:
//	pgr_image(std::string text);	
//	void tst_func();

	pgr_camera(ros::NodeHandle &nh, std::string topic_name_left, std::string topic_name_right, std::string server_name);
	void checkCamera();
	void takeStereoImages(int trial);
	void showStereoImages();
	void waitForEnter();

	cv::Mat image_left, image_right;	// Output taken images
};

#endif
