//
// stereo_image_subscriber.h
//
//
// Created by Alex Zhu on 9/11/13.
//
//

#ifndef ____stereo_image_subscriber__
#define ____stereo_image_subscriber__

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <queue>

#include <boost/bind.hpp>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pgrcamera_driver/TakePicAction.h>

class stereo_image_subscriber{
 public:
  stereo_image_subscriber(ros::NodeHandle nh);
  void take_photos();
  std::queue<cv::Mat> left_images,right_images;
  cv::Mat left_image,right_image;
  bool left_updated,right_updated;
 private:
  pgrcamera_driver::TakePicGoal takepic;
  actionlib::SimpleActionClient<pgrcamera_driver::TakePicAction> ac;
  
  image_transport::ImageTransport it;
  image_transport::Subscriber pic_left;
  image_transport::Subscriber pic_right;
  
  void convert_image(const sensor_msgs::ImageConstPtr& msg,bool left);
};
#endif /* defined(____stereo_image_subscriber__) */
