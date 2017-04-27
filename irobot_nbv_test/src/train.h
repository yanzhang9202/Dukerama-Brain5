#ifndef ____TRAIN_H___
#define ____TRAIN_H___

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

ros::Subscriber robot_vrpn_sub;

float downsample;
float flength, bline;
Eigen::Vector2f cc_left, cc_right;
Eigen::Matrix4f Hc2m_left, Hc2m_right;
Eigen::Matrix3f Rot_c2h, Rot_h2w, Rot_c2w, Rot_w2c;
Eigen::Vector4f q_h;
Eigen::Vector3f ocam_hand, ohand_world, ocam_world;

Eigen::MatrixXf Targets;
Eigen::MatrixXf Waypoint;

void vrpnCallback(const geometry_msgs::TransformStamped msg);

void loadWaypoint();
void loadTarget();
int getNumberofLine(std::string filename);
void readMatrix(std::string filename, int max_line, int max_col, Eigen::MatrixXf &mat);

void getCameraCalib();

// New controller functions and variables
void calcOrder();
Eigen::Matrix3f buildCamGoal(Eigen::Vector3f az);
Eigen::Matrix2f matrix2d(int index1, int index2, Eigen::Matrix3f mat);

Eigen::Vector3f cam_goal, hand_goal, tgt;
Eigen::Matrix3f Rcam_goal, Rhand_goal;
Eigen::Vector2f w_hat;
Eigen::Matrix2f Rg_hat;

int tst_count;

void waitForEnter();

#endif
