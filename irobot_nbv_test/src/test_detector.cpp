#include "detector_lib.cpp"

using namespace cv;
using namespace std;

int main(int argc, char **argv){
  ros::init(argc, argv, "test_detector");
  ros::NodeHandle n;

//// Declare robot name, server name and image topics' names
  string robotname, server_name, topic_name_left, topic_name_right;
  n.getParam("/robot/name", robotname); 
  server_name = robotname+"/take_pic";
  topic_name_left = robotname+"/left/image_rect_color";		// Take raw images or rectified images
  topic_name_right = robotname+"/right/image_rect_color";

//// Load target information, down-sampling rate, circle finder resolution
  int downsample = 24;
  float decimal = 1;

  bool sim = false;

//// Define nbv_detector
  nbv_detector detector(n, topic_name_left, topic_name_right, server_name, downsample, decimal, sim);

  detector.checkCamera();

  detector.waitForEnter();

  int tst_count = 0;
//// Take images and detect targets
  while (ros::ok() && tst_count < 5){
	detector.detectTargets();
	detector.waitForEnter();
	tst_count++;
  }
}

