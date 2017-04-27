#ifndef wpslib_H
#define wpslib_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <geometry_msgs/Point.h>

#define NUM_WPS 5
#define NUM_TARGET 3

Eigen::MatrixXf Waypoints(NUM_WPS, 3);
Eigen::MatrixXf InitTargets(NUM_TARGET, 3);

ros::Publisher wps_pub;

ros::Publisher target_pub;

ros::Subscriber wps_rqst_sub;

ros::Publisher all_sent_pub;

int wps_count = 0;
int wps_request = 0;
int wps_heard = 0;
int tgt_heard = 0;

int wps_done = 0;

int all_sent = 0;

int goal_target = 0;

geometry_msgs::Point wps_msg, tgt_msg, all_sent_msg;

void loadWaypoints();
void loadTargets();
void WpsRqstCallback(const geometry_msgs::Point msg);

#endif  // __WPSLIB_H__
