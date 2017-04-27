#ifndef ___SENSOR_LISTENER_H___
#define ___SENSOR_LISTENER_H___

#include <ros/ros.h>
#include <iostream>
#include <fstream>
//#include <irobot_create_2_1/SensorPacket.h>
#include <SensorPacket.h>
#include <Eigen/Dense>
#include <string.h>

class sensor_listener{
private:
	// Define subscriber and callback function
	void sensorCallback(const irobot_create_2_1::SensorPacket msg);
	ros::Subscriber sensor_sub;
	float distance;
	float angle;
	float current;
	float cliffLeftSignal;

	std::ofstream outfile_sensor;
	double record_current_time, display_current_time;
	double record_interval;
	double display_interval;
public:
	// Constructor
	sensor_listener(ros::NodeHandle &nh, std::string name);

	// Record data from vrpn
	void recordData();
	void closeFile();

	// Display data from vrpn
	void displayData();

	void waitForEnter();
};

#endif
