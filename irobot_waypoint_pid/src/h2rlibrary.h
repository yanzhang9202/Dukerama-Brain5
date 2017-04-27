#pragma once

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>

#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#define PI 3.14159265

// trans_h2r is the transformation matrix from hand's rotation to robot's rotation
//Eigen::MatrixXd trans_h2r(3, 3);

Eigen::VectorXf q_h(4), q_r(4);

Eigen::Matrix3f Rot_h2w, Rot_r2n, Rot_r2w, Rot_n2h;

Eigen::AngleAxisf aa_r;

geometry_msgs::TransformStamped msg_r;

float servo_position, servo_cos, servo_sin;
