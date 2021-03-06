#ifndef ___IC_CONTROLLER_H___
#define ___IC_CONTROLLER_H___

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

class ic_controller{
private:
	// Simulation or experiment
	bool sim;

	// Load robot infomation
	Eigen::MatrixXf robotName;
	bool show_sys_info;

	void loadRobotInfo();
	int getNumberofLine(std::string filename);
	void readMatrix(std::string filename, int max_line, int max_col, Eigen::MatrixXf &mat);

	// Set vrpn subscribers
	std::vector<ros::Subscriber> vrpn;
	std::vector<Eigen::Vector2f> pose;

	void vrpnCallback(const geometry_msgs::TransformStamped::ConstPtr& msg, int i);

	// System initial set up
	int myName;
	int myIndex;
	int count_ngbr;
	Eigen::MatrixXf nodes;
	Eigen::MatrixXi edges;
	Eigen::Vector2i myedge;
	std::vector<std::vector<int> > myngbr;
	ros::Publisher my_pub;
	std_msgs::Float32MultiArray my_msg;

	std::vector<ros::Subscriber> peer_sub;
	std::vector<Eigen::Vector2f> peers_info;

	void loadInitialNum();
	void findmyIndex();
	void loadNodesInfo();
	void loadRobotPoses();
	void findEdges();
	void findmyNeighbour();
	void setPub(ros::NodeHandle &nh);
	void setSub(ros::NodeHandle &nh);
	void peerCallback(const std_msgs::Float32MultiArray::ConstPtr& array, int i);

	// Policy
	Eigen::MatrixXf policy;
	int current_node;

	void loadPolicy();

	// Communication protocals
	bool start_commu;
	int req_flag;
	float num_carry;
	ros::Rate pub_rate;
	bool query_state;
	bool peer_query;
	std::vector<bool> selfrecord;
	std::vector<bool> peerrecord;
	int index_node;
	Eigen::VectorXf data_record;
	std::vector<int> to_who;

	void publishmsg(float query, float num, std::vector<int> ngbr);
	void checkselfFullfilled();
	void checkpeerFullfilled();
	void setrecord();
	void findcurrentNgbr();
	bool checkmsg4me();

	void calcMean();

	// Listen to master
	ros::Subscriber master_sub;
	bool state_flag;

	void masterCallback(const std_msgs::Float32MultiArray::ConstPtr& array);

	// Auxilary function
	std::ofstream outfile_number;

	void recordNumber();

public:
	ic_controller(ros::NodeHandle &nh);

	Eigen::Vector2f waypoint;

//	// Main process
	void setInitialPose();
	void setInitialNextPose();
	void setNextPose();

	void startCommu();

	void closeRecord();

	bool checkMaster();

	// Test functions
	void test_vrpn();

	// Simulation
	void waitForSeconds(double i);
	void waitForEnter();
};

#endif
