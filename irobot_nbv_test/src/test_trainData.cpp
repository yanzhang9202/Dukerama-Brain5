#include "detector_lib.cpp"
#include "waypoint_controller.cpp"
#include "train.cpp"

using namespace cv;
using namespace std;
using namespace Eigen;

int main(int argc, char **argv){
  ros::init(argc, argv, "test_trainData");
  ros::NodeHandle n;

//// Create two windows to show image
  namedWindow("right",CV_WINDOW_NORMAL);
  namedWindow("left",CV_WINDOW_NORMAL);

//// Declare robot name, server name and image topics' names
  string robotname, server_name, topic_name_left, topic_name_right;
  n.getParam("/robot/name", robotname); 
  server_name = robotname+"/take_pic";
  topic_name_left = robotname+"/left/image_rect_color";		// Take raw images or rectified images
  topic_name_right = robotname+"/right/image_rect_color";

//// Load target information, down-sampling rate, circle finder resolution
  int downsample = 24;
  float decimal = 1;

//// Simulation or Experiment
  bool sim = false;
  int dim = 2;
  Vector2f tp, r_init;
  Matrix2f R_init;

//// Simulation starting state
  if (sim) {
	tp << 54.7858, 33.5861;
	r_init << 0, 0;
	R_init << 0, 1, -1, 0;
  }
  Vector2f r = r_init;
  Matrix2f R = R_init;

//// Define detector, localizer and controller
  nbv_detector detector(n, topic_name_left, topic_name_right, server_name, downsample, decimal, sim);

  detector.checkCamera();

  tst_count = 0;

  robot_vrpn_sub = n.subscribe("Brain5Camera/pose", 1, vrpnCallback);

  waypoint_controller waypoint(n);

//// Special setting
  int position, servo_count;
  float origin, dev, target_pos;

  loadTarget();
  loadWaypoint();
  getCameraCalib();

  std::ofstream outfile_pose("/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/Waypoints/handpose.txt");

  waypoint.testVrpn();

  while (ros::ok() && tst_count < NUM_MEAS){

	cout << "Measurement count: " << tst_count+1 << endl << endl;

	calcOrder();

	waypoint.goWaypoint(w_hat, Rg_hat);

        int fd = maestroConnect();
	position = maestroGetPosition(fd, 0);
	origin = ((float)position)/4;
	close(fd);

     	for (dev = -25; dev < 40; dev = dev + 5){
		target_pos = origin + 1000 / 360 * dev;
		if (target_pos > 1050 && target_pos < 1950){
			int fd = maestroConnect();
			maestroSetTarget(fd, 0, target_pos*4);
			servo_count = 0;
			while(GetMovingState(fd,0)){
				if (servo_count == 0) { printf("Moving...\n"); }
				servo_count++;
				close(fd);
			}

			ros::Duration(0.5).sleep();

			detector.detectTargets();

			ros::spinOnce();
		  	for (int i=0;i<3;i++) {
		    		outfile_pose << ohand_world(i) << " ";
		  	}
		  	for (int i=0;i<4;i++) {
		    		outfile_pose << q_h(i) << " ";
		  	}
		  	outfile_pose << "\n";

//			waitForEnter();
		} else {
		   std::cout << "Position wanted is out of Servo Limit!" << std::endl << std::endl;
		}
	}
	tst_count++;
  }
  outfile_pose.close();
  detector.closeRecord();
}
