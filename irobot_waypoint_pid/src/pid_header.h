#pragma once

#include "ros/ros.h"
#include <stdio.h>
#include <string>
#include <iostream>

using namespace std;

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <std_msgs/Bool.h>

bool mission_flag = false;

float Kp_speed = 0.5;
float Ki_speed = 0;
float Kd_speed = 0;

float Kp_head = 2;
float Ki_head = 0;
float Kd_head = 0;

double prev_time = 0.0;
float delta_t = 0.0;
float error_integral_speed = 0.0, error_integral_head = 0.0;

float cutoff_frequency= -1.0; // Cutoff frequency for the derivative calculation in Hz. Negative-> Has not been set by the user yet, so use a default.

float ul_speed=0.1, ll_speed=-0.1; // Upper and lower speed saturation limits
float ul_head=0.3, ll_head=-0.3; // Upper and lower heading saturation limits
float anti_w = 0.5; // Anti-windup term. Limits the absolute value of the integral term.

Eigen::VectorXf error_head(3), error_speed(3);
Eigen::VectorXf filtered_error_speed(3), filtered_error_head(3);
Eigen::VectorXf error_deriv_speed(3), error_deriv_head(3);
Eigen::VectorXf filtered_error_deriv_speed(3), filtered_error_deriv_head(3);
int loop_counter_speed = 0, loop_counter_head = 0; // Counts # of times through the control loop. Used to start taking a derivative after 2 rounds

Eigen::VectorXf waypoint(3), waypoint_r(3), position_r(3), waypoint_r2D(2);
Eigen::VectorXf q_r(4);
Eigen::Matrix3f Rot_r2w, Rot_w2r;

geometry_msgs::Twist  u_msg;

////////////////
// Functions
////////////////
void check_user_input(int& argc, char** argv);
void robotposeCallback(const geometry_msgs::TransformStamped msg);
void waypointCallback(const geometry_msgs::Point wp_msg);

