#include "detector_lib.cpp"
#include "localizer_lib.cpp"

using namespace cv;
using namespace std;

void waitForEnter();

int main(int argc, char **argv){
  ros::init(argc, argv, "test_detector");
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

  bool sim = false;

//// Define nbv_detector and localizer
  nbv_detector detector(n, topic_name_left, topic_name_right, server_name, downsample, decimal, sim);

  nbv_localizer localizer(n, downsample, detector.num_targets, sim);

//  localizer.test_vrpn();

  detector.checkCamera();

  int tst_count = 0;
//// Take images and detect targets
  while (ros::ok() && tst_count < 1){
	detector.detectTargets();
	waitForEnter();
	localizer.calcMeasurements(detector.target_ctr_left, detector.target_ctr_right, detector.flag_detect);
	tst_count++;
  }
}

void waitForEnter(){
	char enter;
	cin >> enter;
	cin.clear();
	cin.sync();
}
