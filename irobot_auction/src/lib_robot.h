#ifndef ___LIB_ROBOT_H_____
#define ___LIB_ROBOT_H_____

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>

#include "pid_controller.cpp"

#include "maestro_serial_driver.cpp"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

#include <stdio.h>
#include <math.h>

#include <geometry_msgs/Twist.h>

// Robot Class Variable
#define PI 3.14159265

#define Camera_Orientation_Error_Tolerance 1.0
#define Base_Orientation_Error_Tolerance 1.5
#define Waypoint_Error_Tolerance 0.025

//ros::Subscriber robot_servo_sub = n.subscribe("Brain5/servo_position", 1, servoCallback);
//ros::Subscriber robot_vrpn_sub = n.subscribe("Brain5Camera/pose", 1, vrpnCallback);
//ros::Publisher base_pub = n.advertise<geometry_msgs::Twist>("Brain5/cmd_vel", 1);
//ros::Subscriber robot_servo_sub;
ros::Subscriber robot_vrpn_sub;
ros::Publisher base_pub;

int raw_input = 0;

float servo_position, servo_cos, servo_sin, error_angle;
double start_time = 0, sec = 7, duration = 0, waitforWpt = 10, waitforServo = 5, waitforBase = 10;
float flag_forward_backward = 1; // 1 - Robot moving forward to the waypoint; -1 - Robot moving backward to the waypoint.
float BaseError;
int counter = 0;
bool exceedLimit = false, flag_exceedLimit = false;

Eigen::VectorXf q_h(4), ocam_hand(3), ocam_world(3), ohand_world(3), tgt_world(3), tgt_cam(3), wpt2tgt(3), ocam2wpt(3), ocam2wpt2D(2), waypointError(2);
Eigen::VectorXf wpt2tgt_robot(3), ocam2wpt_world(3), ocam2wpt2D_world(2), q_cam(4);
Eigen::Matrix3f Rot_h2w, Rot_c2h, Rot_c2w, Rot_w2c, Rot_r2n, Rot_n2h, Rot_r2w, Rot_w2r;

Eigen::VectorXf servo_center_world(3), base_center_world(3), ohand2servo_hand(3), bctr2ocam(3), bctr2ocam2D(2), bctr2wpt(3), bctr2wpt2D(2);

pid_controller base_pid(1.0, 0.2, 0.1, 0.3, -0.3 , Base_Orientation_Error_Tolerance/180*PI); // input : [Kp, Ki, Kd, ul, ll, tolerance(in rad)];
pid_controller wpt_dist_pid(0.3, 0, 0, 0.2, -0.2, Waypoint_Error_Tolerance);
pid_controller wpt_ori_pid(0.5, 0, 0, 0.4, -0.4, Base_Orientation_Error_Tolerance/180*PI);

geometry_msgs::Twist  u_msg;

bool flag_i_turned = false;
bool flag_beat_in_transit = false;
int flag_getCurrentPosition;

// Robot Class function

void vrpnCallback(const geometry_msgs::TransformStamped msg);
void servoCallback(const geometry_msgs::Point msg);

void adjustBase();
void goingWaypoint(float Wpt_Tolerance);
void ifExceedServoLimit();
void getServoError();
bool sendServoInput();
float getBaseError();
void ifClosetoWpt();
void stopBase();

void sendServoSpeed();
void initialServoTarget();
void BaseBackUp();

void getServoPosition();

bool checkBaseBoundary();
void ifWptclosetoBase();

void waitForEnter();
void goWaypoints();

void compromisesendServoInput();

void waitForServoStill();

int robot_wpt_count;

Eigen::VectorXf getWaypointError();

// Variable communicated between commander and robot

Eigen::VectorXf waypoint(6);
Eigen::Vector3f target;

bool robot_got_mission = false;

int which_target;

struct NearestEntry{
  int which_target;
  int which_entry;
  float distance;
  float bid;
};

NearestEntry my_chosen_entry;

Eigen::MatrixXf current_waypointlist, next_waypointlist;
Eigen::VectorXf last_waypoint_in_current_list(2);

std::ofstream outfile_sim("/home/dukerama/hydro_catkin/src/irobot_auction/include/positions/Brain5_SimPoses.txt");
std::ofstream outfile_cam("/home/dukerama/hydro_catkin/src/irobot_auction/include/positions/Brain5_CamPoses.txt");
std::ofstream outfile_hand("/home/dukerama/hydro_catkin/src/irobot_auction/include/positions/Brain5_HandPoses.txt");
std::ofstream outfile_target("/home/dukerama/hydro_catkin/src/irobot_auction/include/positions/Brain5_target.txt");
std::ofstream outfile_visitHist("/home/dukerama/hydro_catkin/src/irobot_auction/include/positions/Brain5_visitHistory.txt");
std::ofstream outfile_handTime("/home/dukerama/hydro_catkin/src/irobot_auction/include/positions/Brain5_HandPosesTime.txt");

#endif
