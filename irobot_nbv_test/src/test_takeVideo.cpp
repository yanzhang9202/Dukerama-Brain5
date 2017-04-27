#include "image_lib.cpp"

using namespace cv;
using namespace std;

int main(int argc, char **argv){
  ros::init(argc, argv, "test_takeVideo");
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

//// Declare image variable
  Mat image_left,image_right; 

//// Define pgr_image class
  pgr_camera camera(n, topic_name_left, topic_name_right, server_name);

//// Check camera works normally
  camera.checkCamera();

//// Video (approximate) frame rate
  ros::Rate freq(10);
  int trial = 1;

  while(ros::ok()){
	camera.takeStereoImages(trial);
	camera.showStereoImages();
	freq.sleep();
  }

  return 0;
}
