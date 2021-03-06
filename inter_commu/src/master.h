#ifndef ___MASTER_H___
#define ___MASTER_H___

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <boost/bind.hpp>

#include <math.h>
#include <stdlib.h>

#include "definition.h"

class ic_master{
private:
	// Load robot infomation
	Eigen::MatrixXf robotName;
	bool show_sys_info;

	void loadRobotInfo();
	int getNumberofLine(std::string filename);
	void readMatrix(std::string filename, int max_line, int max_col, Eigen::MatrixXf &mat);

	// Load initial number
	Eigen::MatrixXf num_carry;
	float goal;

	void loadInitialNum();

	// Publisher and Subscriber
	ros::Rate sub_rate;
	std::vector<int> to_who;
	std::vector<Eigen::Vector2f> peers_info;
	Eigen::MatrixXf error;
	Eigen::MatrixXf old_num;
	std_msgs::Float32MultiArray my_msg;
	
	ros::Publisher my_pub;
	std::vector<ros::Subscriber> my_sub;

	void peerCallback(const std_msgs::Float32MultiArray::ConstPtr& array, int i);
	bool checkError();	
	void publishmsg(float query);

	bool state_flag;
public:
	ic_master(ros::NodeHandle &nh);
	
	void supervise();
};

#endif
