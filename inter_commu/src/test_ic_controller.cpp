#include "ic_controller.cpp"

using namespace std;

int main(int argc, char **argv){
  ros::init(argc, argv, "test_detector");
  ros::NodeHandle n;

  ic_controller controller(n);

  controller.test_vrpn();

//  int tst_count = 0;
////// Take images and detect targets
//  while (ros::ok() && tst_count < 1){
//	tst_count++;
//  }
}

