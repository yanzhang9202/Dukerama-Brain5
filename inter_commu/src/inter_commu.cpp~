#include "ic_controller.cpp"
#include "waypoint_controller.cpp"

using namespace std;

int main(int argc, char **argv){
  ros::init(argc, argv, "inter_communication");
  ros::NodeHandle n;

  ic_controller controller(n);

  waypoint_controller waypoint(n);

  controller.waitForEnter();

  ros::Duration(90.00).sleep();

  controller.setInitialPose();

  waypoint.goWaypoint(controller.waypoint);

//  controller.waitForSeconds(3);

  controller.setInitialNextPose();

  waypoint.goWaypoint(controller.waypoint);

//  controller.waitForSeconds(3);

  int count = 0;

  while(ros::ok() && (!controller.checkMaster())){
	controller.startCommu();

	if (!controller.checkMaster()){
		controller.setNextPose();

  		waypoint.goWaypoint(controller.waypoint);

//  		controller.waitForSeconds(1);

		count++;
	}
  }

  controller.closeRecord();

  ros::Duration(60).sleep();
  
  cout << "Job finished!" << endl << endl;
}

