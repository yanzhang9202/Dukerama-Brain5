#include "detector_lib.cpp"
#include "localizer_lib.cpp"
#include "nbv_controller_V2.cpp"

using namespace cv;
using namespace std;
using namespace Eigen;

int main(int argc, char **argv){
  ros::init(argc, argv, "test_nbv");
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
  float decimal = 0;

//// Simulation or Experiment
  bool sim = true;
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

  string obj("supreme");
  nbv_controller controller(n, detector.num_targets, downsample, obj, sim, dim);

////  detector.checkCamera();

  int tst_count = 0;
// Take images and detect targets

// Show robot initial pose
cout << "Initial pose: " << endl;
cout << "r: " << r.transpose() << endl;
cout << "R: " << endl << R << endl << endl;

  while (ros::ok() && tst_count < 34){
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
		detector.detectTargets();

		// Calculate single measurements and covariance matrix, KF filtering
		localizer.calcMeasurements(detector.target_ctr_left, detector.target_ctr_right, detector.flag_detect);

		// NBV controller, decide where to go next
		controller.calcNBV(localizer.targets, detector.flag_detect);
	}
	tst_count++;
  }
}
