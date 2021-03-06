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
#include "maestro_serial_driver.cpp"

#define Base_Orientation_Error_Tolerance 1.0
#define Waypoint_Error_Tolerance 0.02
#define Freeze_Tolerance 0.000000001

class waypoint_controller{
public:
	waypoint_controller(ros::NodeHandle &nh);

	// Receive stereo center and orient from NBV/Auction controller
	void goWaypoint(Eigen::Vector2f w_hat_order, Eigen::Matrix2f Rg_hat_order);	// Receive goal pose and transform to robot base frame

	// Test functions
	void testServo();
	void testVrpn();
	void testBaseInfo();
	void testCalcAngle();
	void testRotatePID();
	void testForwardPID();
	void testgoWaypoint();

	// Auxilary function
	void waitForEnter();

private:
	// Servo Function and variables
	void sendServoSpeed(float speed);
	void initialServoTarget();
	void getServoPosition();
	void sendServoTarget(float ms);

	float servo_position, servo_angle;

	// VRPN function and variables
	void vrpnCallback(const geometry_msgs::TransformStamped msg);
	
	ros::Subscriber vrpn_sub;
	Eigen::Matrix4f Hc2m_left, Hc2m_right;
	Eigen::Matrix3f Rot_c2h, Rot_h2w, Rot_c2w, Rot_w2c;
	Eigen::Vector4f q_h;
	Eigen::Vector3f ocam_hand, ohand_world, ocam_world;

	// Load Base information
	void loadBase();
	Eigen::MatrixXf loadMatrix(int line, int col, std::string filename);

	bool show_Base_info;
	Eigen::Matrix2f R_sb;
	Eigen::Vector2f r_sb, r_hs;

	// Real time base pose information
	void getBasePose();
	Eigen::Matrix2f angle2rot(float angle);
	Eigen::Matrix2f matrix2d(int index1, int index2, Eigen::Matrix3f mat);

	Eigen::Matrix2f R_bw, R_stheta, R_hw;
	Eigen::Vector2f p_hand, p_base;

	// Calculate angle the base and servo need to rotate
	void trans2base();
	bool calcAngle();

	Eigen::Vector2f w_hat;
	Eigen::Matrix2f Rg_hat;
	Eigen::Vector2f w, a;
	Eigen::Matrix2f Rg, R_b_delta, R_s_delta;
	float b, angle_a, angle_rot, angle_b1, angle_b2, theta_b, angle_s;
	bool doable;

	// PID controller
	void calcRotateError();
	void calcForwardError();
	void PID_rotate();
	void PID_forward(bool rotate);
	void stopBase();
	void backwardBase();

	Eigen::Matrix2f R_basegoal;
	Eigen::Vector2f ax_goal, ex, ax_base, bearing;
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
