#ifndef ___SENSOR_LISTENER_CPP___
#define ___SENSOR_LISTENER_CPP___

#include "sensor_listener.h"

using namespace std;
using namespace Eigen;

sensor_listener::sensor_listener(ros::NodeHandle &nh, string name):
outfile_sensor("/home/dukerama/hydro_catkin/src/irobot_odometry/include/sensor.txt"){
	cout << "Initialize sensor listener..." << endl;

	string topicname = name + "/sensorPacket";
	sensor_sub = nh.subscribe<irobot_create_2_1::SensorPacket>(topicname.c_str(), 1, &sensor_listener::sensorCallback, this);

	distance = 1;
	angle = 1;

	record_interval = 0.5; // unit: s
	record_current_time = ros::Time::now().toSec();
	display_interval = 0.5;	// unit: s
	display_current_time = ros::Time::now().toSec();
}

void sensor_listener::sensorCallback(const irobot_create_2_1::SensorPacket msg){
	// receive data
	distance = (float)msg.distance;
	angle = (float)msg.angle;
	current = (float)msg.current;
	cliffLeftSignal = (float)msg.cliffLeftSignal;	
}

void sensor_listener::recordData(){
//	if (ros::Time::now().toSec() - record_current_time > record_interval){
//		outfile_sensor << "\n";
//		record_current_time = ros::Time::now().toSec();
//	}
}

void sensor_listener::closeFile(){
	outfile_sensor.close();
}

void sensor_listener::displayData(){
	if (ros::Time::now().toSec() - display_current_time > display_interval){
		cout << "Distance: " << distance;
		cout << "   Angle: " << angle;
		cout << "   Current: " << current;
		cout << "   cliffLeftSignal: " << cliffLeftSignal << endl << endl;
		display_current_time = ros::Time::now().toSec();
	}
}

void sensor_listener::waitForEnter(){
	char enter;
	cout << "Type any key + enter to continue: ";
	cin >> enter;
	cin.clear();
	cin.sync();
}

#endif
