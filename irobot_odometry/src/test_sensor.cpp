#include "sensor_listener.cpp"

using namespace std;

int main(int argc, char **argv){
	ros::init(argc, argv, "test_sensor");
	ros::NodeHandle n;

	string objectName = "Brain5";
	sensor_listener listener(n, objectName);
	listener.displayData();

	ros::Rate loop_rate(10);
	while(ros::ok()){
		ros::spinOnce();
//		listener.recordData();
		listener.displayData();
		loop_rate.sleep();
	}
  	listener.closeFile();
}
