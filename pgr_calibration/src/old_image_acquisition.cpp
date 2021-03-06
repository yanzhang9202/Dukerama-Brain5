#include "iostream"
#include "fstream"

#include "vrpnNode.cpp"
#include "CameraTrackable.cpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pgrcamera_driver/TakePicAction.h>
#include <boost/bind.hpp>

#include "FlyCapture2.h"

#define SERIAL_LEFT 13200942
#define SERIAL_RIGHT 13200944

using namespace FlyCapture2;
using namespace cv;

void waitFor(unsigned int secs){
	unsigned int retTime;
	retTime = time(0) + secs;
	while (time(0) < retTime);
}
void waitForEnter(){
	string Enter;
	std::cin >> Enter;
}
void convert_image(const sensor_msgs::ImageConstPtr& msg,Mat* image){
  sensor_msgs::Image imag=*msg;
  //std::cout << "Image time is: " << imag.header.stamp << std::endl;
  cv_bridge::CvImagePtr cv_ptr;
  //printf("Got image\n");
  ros::spinOnce();
    try
      {
        cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");//sensor_msgs::image_encodings::BGR8);                                                                                                                  
       }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    cv::Mat image_out=cv_ptr->image;
    image_out.copyTo(*image);//left_images.push(image_out);                                                                         
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "inputScanner");
  ros::NodeHandle n;
  cv::namedWindow("right",CV_WINDOW_NORMAL);
  cv::namedWindow("left",CV_WINDOW_NORMAL);
  
  
  std::ofstream outfile("/home/dukerama/catkin_ws/src/pgr_calibration/include/positions/Positions.txt");

  actionlib::SimpleActionClient<pgrcamera_driver::TakePicAction> ac("take_pic", true); //First argument in ac constructor is the server to connect to

  cv::Mat image_left,image_right;

  std::string myself="Brain5Camera";
  CameraTrackable* me=new CameraTrackable();

  vrpnNode vrpn(n,me,myself);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber pic_left=it.subscribe("/stereo/left/image_rect_color",1,boost::bind(convert_image,_1,&image_left));
  image_transport::Subscriber pic_right=it.subscribe("/stereo/right/image_rect_color",1,boost::bind(convert_image,_1,&image_right));

  pgrcamera_driver::TakePicGoal goal;
  goal.acquire=true;
  
  ros::Rate loop_rate(60);

  bool updated=false;
  bool started=false;
  int count=600;
  char exit=-1;

  int num_ima=0;

  ros::Time now=ros::Time::now();
  ros::Time last=ros::Time::now();

  Eigen::Vector3d myPos, myLastPos;
  Eigen::Vector4d myQuat;
  Eigen::Matrix3d myRot, myLastRot;

  int update_cnt = 1;
	

  while (ros::ok() && num_ima<4 /*&& exit%256!=27/* && exit!=/*27*/)
    { 
                                                                                                       
     if (!updated){
	ac.waitForServer();
	ac.sendGoal(goal);                                                                                                                   
	bool finished_before_timeout = ac.waitForResult(ros::Duration(0.0));
	printf("\n UpdateLoopin--  %d \n", update_cnt);
	update_cnt++;
      }


	//Capture rotation and translation
	myPos=me->getCameraPosition();
        myQuat = me->getQuaternion();
      

      //myRot=me->getCameraRotation();

      if (!image_left.empty() && !image_right.empty()){
	updated=true;

	imshow("right",image_right);
	imshow("left",image_left);
	exit=waitKey(20);
	now=ros::Time::now();

	
	ac.sendGoal(goal);
	ros::spinOnce();

	char filename_left[512];
	sprintf( filename_left, "/home/dukerama/catkin_ws/src/pgr_calibration/include/calib_images/left%d.bmp", count );
	  
	char filename_right[512];
	sprintf( filename_right, "/home/dukerama/catkin_ws/src/pgr_calibration/include/calib_images/right%d.bmp", count );
	
	    
	// Save the image. If a file format is not passed in, then the file
	// extension is parsed to attempt to determine the file format.
	    
	if ((now-last).toSec()>1){

	 	 std::cout << "My position is: \n" << myPos << std::endl;
		 std::cout << "My Quaternion is: \n" << myQuat << std::endl;
		 last=now;
		 imwrite(filename_left,image_left);
		 imwrite(filename_right,image_right);
		  
		  
		 for (int i=0;i<3;i++){
		    outfile << myPos(i) << " ";
		  }

		 for (int i=0;i<4;i++){
		    outfile << myQuat(i) << " ";
		 }

		  outfile << "\n";
		  
		  count++;
		printf("Count=%d %d %d %d %d %d %d %d\n"/*Press space to take a photo or escape to quit\n"*/,count,count,count,count,count,count,count,count);
		num_ima++;

		//Set up station
		std::cout<<"\n Set up your station: ";
		waitForEnter();	
	}
	
	  
      }
	
      ros::spinOnce();
    }
  
  printf( "\nFinished grabbing images\n" );
  outfile.close();
  
  return 0;
}
