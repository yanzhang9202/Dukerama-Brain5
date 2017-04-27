#ifndef _PGRCAMERA_H
#define _PGRCAMERA_H

#include "pgrcamera_driver/TakePicAction.h"
#include <actionlib/server/simple_action_server.h>
#include <FlyCapture2.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#define SERIAL_LEFT 13200942
#define SERIAL_RIGHT 13200944

//#define SERIAL_LEFT 13344924
//#define SERIAL_RIGHT 13344950

using namespace FlyCapture2;
typedef actionlib::SimpleActionServer<pgrcamera_driver::TakePicAction> Server;

class pgr_camera
{
 protected:
  //ros::NodeHandle n;
  Server server;
  image_transport::ImageTransport it_left;
  image_transport::ImageTransport it_right;
  camera_info_manager::CameraInfoManager cam_info_left;
  camera_info_manager::CameraInfoManager cam_info_right;
  pgrcamera_driver::TakePicResult result;
  
 public:
  pgr_camera(ros::NodeHandle n_priv,ros::NodeHandle n_left, ros::NodeHandle n_right);
  BusManager busMgr;
  Camera** cameras;
  uint numCameras;
  int connect_cameras();
  int start_cameras();
  int start_stream();
  int disconnect_cameras();
  void execute(/*const pgrcamera_driver::TakePicGoalConstPtr& goal*/);

 private:
  image_transport::CameraPublisher left_pub;
  image_transport::CameraPublisher right_pub;
  sensor_msgs::CameraInfo cam_left_info;
  sensor_msgs::CameraInfo cam_right_info;
  //  sensor_msgs::Image downsample(const sensor_msgs::Image& msg);
  Mode k_fmt7Mode;
  PixelFormat k_fmt7PixFmt;
  std::string left_url;
  std::string right_url;
  Error error;
  inline void PrintError(Error error);
};
#endif
