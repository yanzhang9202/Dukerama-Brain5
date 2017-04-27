#include "detector_lib.cpp"
#include "localizer_lib.cpp"
#include "nbv_controller_V2.cpp"
#include "waypoint_controller.cpp"

using namespace cv;
using namespace std;
using namespace Eigen;

int main(int argc, char **argv){
  ros::init(argc, argv, "test_nbv_V2");
  ros::NodeHandle n;

//// Create two windows to show image
//  namedWindow("right",CV_WINDOW_NORMAL);
//  namedWindow("left",CV_WINDOW_NORMAL);

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

  nbv_localizer localizer(n, downsample, detector.num_targets, sim);

//  string obj("supreme");
  string obj("central");
  nbv_controller controller(n, detector.num_targets, downsample, obj, sim, dim);

  detector.checkCamera();

  int tst_count = 0;
// Take images and detect targets

if (sim) {
	// Show robot initial pose
	cout << "Initial pose: " << endl;
	cout << "r: " << r.transpose() << endl;
	cout << "R: " << endl << R << endl << endl;
}

  waypoint_controller waypoint(n);

  waypoint.testVrpn();

//// If lose target
//  int trial = 3;
//  int count_trial = 0;

//  ros::Duration(45).sleep();

  // Set and achieve initial pose
//  r << -0.8946, -0.1596;
//  R <<  0.7547, -0.6559,
//	0.6561,  0.7545;

//  r << -1.2295, -0.0944;
//  R <<  0.8134, -0.5813,
//	0.5815,  0.8135; 

//  r << -0.9443, -0.3113;
//  R <<  0.7496, -0.6618,
//	0.6618,  0.7497;

//  r << -1.2445, 0.0522;
//  R <<  0.9907, 0.1335,
//       -0.1339, 0.9909;

//  r << -0.8218, 0.2551;
//  R <<  0.9822, 0.1878,
//       -0.1877, 0.9822;

//  r << -1.1668, 0.1298;
//  R <<  0.9809, 0.1925,
//       -0.1928, 0.9812;

  r << -0.8024, 0.0953;
  R <<  0.9984, 0.0568,
       -0.0568, 0.9984;

  waypoint.goWaypoint(r, R);
  controller.waitForEnter();

  while (ros::ok() && tst_count < NUM_MEAS){
	// Take stereo images and detect targets
	if (sim){
		cout << "Simulation count: " << tst_count+1 << endl << endl;

		detector.simDetect(r, R, tp);
//		// Test simDetect;
//		cout << "target center on left image: " << detector.target_ctr_left[tst_count].transpose() << endl;		
//		cout << "target center on right image: " << detector.target_ctr_right[tst_count].transpose() << endl;

		localizer.simMeasurements(detector.target_ctr_left, detector.target_ctr_right, r, R);

		controller.getSimPose(r, R);
		controller.calcNBV(localizer.targets, detector.flag_detect);
		// This should be accomplished by robots
		r = controller.r_new;
		R = controller.R_new;
//		controller.waitForEnter();

	} else {
		controller.recordPose();

		cout << "Measurement count: " << tst_count+1 << endl << endl;

		detector.detectTargets();

//		if (controller.checkflag(detector.flag_detect)){
//			controller.scanCam();
//			detector.detectTargets();
//		}

//		controller.waitForEnter();

		// Calculate single measurements and covariance matrix, KF filtering
		localizer.calcMeasurements(detector.target_ctr_left, detector.target_ctr_right, detector.flag_detect);

//		controller.waitForEnter();

		// NBV controller, decide where to go next
		controller.calcNBV(localizer.targets, detector.flag_detect);

//		controller.waitForEnter();

		waypoint.goWaypoint(controller.r_hand, controller.R_hand);
	}
	tst_count++;
  }
}
