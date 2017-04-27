//#include "ic_controller.cpp"
#include "waypoint_controller.cpp"

using namespace std;

int main(int argc, char **argv){
  ros::init(argc, argv, "test_goWaypoint");
  ros::NodeHandle n;

  waypoint_controller waypoint(n);

  waypoint.test_goWaypoint();
  
  cout << "Job finished!" << endl << endl;
}
