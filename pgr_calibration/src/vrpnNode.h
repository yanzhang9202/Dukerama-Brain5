//
//  vrpnNode.h
//  
//
//  Created by Alex Zhu on 9/11/13.
//
//

#ifndef ____vrpnNode__
#define ____vrpnNode__

#include <ros/ros.h>

#include <Eigen/Dense>

#include <geometry_msgs/TransformStamped.h>

#include <boost/bind.hpp>

#include "CameraTrackable.cpp"

#include <iostream>

class vrpnNode{
public:
    vrpnNode(ros::NodeHandle n,CameraTrackable* object, std::string myself);
    bool isUpdated();
    CameraTrackable* trackable;
private:
    bool pos_updated;
    ros::Subscriber vrpn_tracker;
    void update_position(const geometry_msgs::TransformStamped msg);
};

#endif /* defined(____vrpnNode__) */
