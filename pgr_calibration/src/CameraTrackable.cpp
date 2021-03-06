//
//  Trackable.cpp
//  
//
//  Created by Alex Zhu on 9/11/13.
//
//

#ifndef ___CameraTrackable_cpp
#define ___CameraTrackable_cpp

#include "CameraTrackable.h"

CameraTrackable::CameraTrackable(){
  position=Eigen::Vector3d::Zero();
  rotation=Eigen::Matrix3d::Zero();
  quaternion = Eigen::Vector4d::Zero();
  servo_angle=0;
  position_shift << 0,0,0.045;
  updated=false;
}

CameraTrackable CameraTrackable::copy(){
  CameraTrackable temp;
  temp.setOrientation(timestamp,position,rotation,quaternion);
  return temp;
}

void CameraTrackable::setOrientation(ros::Time stamp,Eigen::Vector3d pos, Eigen::Matrix3d rot, Eigen::Vector4d quat){
  timestamp=stamp;
  position=pos;
  rotation=rot;
  quaternion = quat;
  updated=true;
}

ros::Time CameraTrackable::getTimeStamp(){
  return timestamp;
}

Eigen::Vector3d CameraTrackable::getPosition(){
  return position;
}

Eigen::Vector4d CameraTrackable::getQuaternion(){
  return quaternion;
}

Eigen::Matrix3d CameraTrackable::getRotation(){
  return rotation;
}

Eigen::Vector3d CameraTrackable::getCameraPosition(){
  Eigen::Matrix3d my_rotation=getCameraRotation();//rotation_shift;
  //rotation_shift<<cos(servo_angle),0,sin(servo_angle),0,1,0,-sin(servo_angle),0,cos(servo_angle);
  
  //return position+my_rotation*position_shift;
   return position;//changed alex's code here	
}

Eigen::Matrix3d CameraTrackable::getCameraRotation(){
 /* Eigen::Matrix3d servo_rotation,rotation_shift;
  servo_rotation<<cos(servo_angle),-sin(servo_angle),0,sin(servo_angle),cos(servo_angle),0,0,0,1;
  rotation_shift<<0,-1,0,0,0,1,1,0,0;
  //return rotation_shift*servo_rotation*rotation_shift.transpose()*rotation;*/
  return rotation;//changed alex's code here
}

void CameraTrackable::updateAngle(std_msgs::Float64 angle){
  //std::cout << "Camera angle updated " << angle << std::endl;
  double angle_in=angle.data;
  
  servo_angle=angle_in;//(angle_in/4-1496)*PI/504;
}

bool CameraTrackable::isUpdated(){
  return updated;
}

#endif
