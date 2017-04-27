#ifndef __WAYPOINT_CONTROLLER_H___
#define __WAYPOINT_CONTROLLER_H___

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <geometry_msgs/TransformStamped.h>
#include <fstream>
#include <string>
#include <stdio.h>
#include <math.h>
#include <geometry_msgs/Twist.h>

#include "pid_controller.cpp"
//#include "maestro_serial_driver.cpp"

#define Base_Orientation_Error_Tolerance 1.0
#define Waypoint_Error_Tolerance 0.05
#define Freeze_Tolerance 0.000000001

class waypoint_controller{
public:
	waypoint_controller(ros::NodeHandle &nh);

	// Receive stereo center and orient from NBV/Auction controller
	void goWaypoint(Eigen::Vector2f w_hat_goal);

	// Test function
	void test_goWaypoint();

	// Auxilary function
	void waitForEnter();

private:

	// VRPN function and variables
	void vrpnCallback(const geometry_msgs::TransformStamped msg);
	
	ros::Subscriber vrpn_sub;
	Eigen::Matrix3f Rot_h2w;
	Eigen::Vector4f q_h;
	Eigen::Vector3f ohand_world;

	// Load Base information
	void loadBase();
	Eigen::MatrixXf loadMatrix(int line, int col, std::string filename);

	bool show_Base_info;
	Eigen::Matrix2f R_sb;

	// Real time base pose information
	void getBasePose();
	Eigen::Matrix2f matrix2d(int index1, int index2, Eigen::Matrix3f mat);

	Eigen::Matrix2f R_bw, R_hw;
	Eigen::Vector2f p_hand;

	// Calculate angle the base and servo need to rotate
	Eigen::Vector2f w_hat;

	// PID controller
	void calcRotateError();
	void calcForwardError();
	void PID_rotate();
	void PID_forward(bool rotate);
	void stopBase();
	void backwardBase();

	Eigen::Matrix2f R_basegoal;
	Eigen::Vector2f ax_goal, ax_base, bearing;
	float err_rad;
	geometry_msgs::Twist  u_msg;

	pid_controller rotate_pid;
	pid_controller forward_pid;
	ros::Publisher base_pub;
	float bearing_angle;

	// Detect system freezing
	bool detectFreeze();
};

#endif
