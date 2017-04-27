#ifndef ___VRPN_LISTENER_H___
#define ___VRPN_LISTENER_H___

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <string.h>

class vrpn_listener{
private:
	// Define subscriber and callback function
	void vrpnCallback(const geometry_msgs::TransformStamped msg);
	ros::Subscriber vrpn_sub;
	Eigen::Vector4f quaternion;
	Eigen::Vector3f position;

	std::ofstream outfile_vrpn;
	double record_current_time, display_current_time;
	double record_interval;
	double display_interval;
public:
	// Constructor
	vrpn_listener(ros::NodeHandle &nh, std::string name);

	// Record data from vrpn
	void recordData();
	void closeFile();

	// Display data from vrpn
	void displayData();
};

#endif
