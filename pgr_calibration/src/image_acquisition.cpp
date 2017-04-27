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

//#define SERIAL_LEFT 13344924
//#define SERIAL_RIGHT 13344950

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
  
  
  std::ofstream outfile("/home/dukerama/hydro_catkin/src/pgr_calibration/include/positions/Positions.txt");

  actionlib::SimpleActionClient<pgrcamera_driver::TakePicAction> ac("Brain5/take_pic", true); //First argument in ac constructor is the server to connect to

  cv::Mat image_left,image_right;

  std::string myself="Brain5Camera";
  CameraTrackable* me=new CameraTrackable();

  vrpnNode vrpn(n,me,myself);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber pic_left=it.subscribe("/Brain5/left/image_rect_color",1,boost::bind(convert_image,_1,&image_left));
  image_transport::Subscriber pic_right=it.subscribe("/Brain5/right/image_rect_color",1,boost::bind(convert_image,_1,&image_right));

  pgrcamera_driver::TakePicGoal goal;
  goal.acquire=true;
  
  ros::Rate loop_rate(60);

  bool updated=false;
  bool started=false;
  int count=601;
  char exit=-1;

  int num_ima=0;

  ros::Time now=ros::Time::now();
  ros::Time last=ros::Time::now();

  Eigen::Vector3d myPos, myLastPos;
  Eigen::Vector4d myQuat;
  Eigen::Matrix3d myRot, myLastRot;

  int update_cnt = 1;
  
	
  std::cout<<"\n Initiating Camera... \n";

  //ping camera driver and wait for it to start
  while (ros::ok() && !updated)
  {
	ac.waitForServer();
	ac.sendGoal(goal);

	if (!image_left.empty() && !image_right.empty())
	{
		imshow("right",image_right);
		imshow("left",image_left);
		cv::waitKey(30);
		updated = true;

	}
	
	ros::spinOnce();
	loop_rate.sleep();
  }

  image_left.release(); image_right.release();

  while (ros::ok() && num_ima<20)
    {
	//Set up station
	std::cout<<"\n Set up your station: ";
	std::cout<<num_ima;
	waitForEnter();
	
    //take picture
    while (image_left.empty() || image_right.empty()) // No matter what first image is taken, dropped it. Because A queue size of 2 somewhere keeps sending old images.
    {
    	ac.sendGoal(goal);
    	ac.waitForResult();
    	ros::spinOnce();
	loop_rate.sleep();
    }
    image_left.release(); image_right.release();

    while (image_left.empty() || image_right.empty()) // No matter what first image is taken, dropped it. Because A queue size of 2 somewhere keeps sending old images.
    {
    	ac.sendGoal(goal);
    	ac.waitForResult();
    	ros::spinOnce();
	loop_rate.sleep();
    }
    image_left.release(); image_right.release();

    while (image_left.empty() || image_right.empty())
    {
    	ac.sendGoal(goal);
    	ac.waitForResult();
    	ros::spinOnce();
	loop_rate.sleep();
    }

	//capture vrpn position and quaternion
	myPos=me->getPosition();
      	myQuat=me->getQuaternion();

	//show image to image window
	imshow("right",image_right);
	imshow("left",image_left);
	cv::waitKey(30);
	
	//write image to file
	char filename_left[512];
	sprintf( filename_left, "/home/dukerama/hydro_catkin/src/pgr_calibration/include/calib_images/left%d.bmp", count );
	char filename_right[512];
	sprintf( filename_right, "/home/dukerama/hydro_catkin/src/pgr_calibration/include/calib_images/right%d.bmp", count );
	imwrite(filename_left,image_left);
	imwrite(filename_right,image_right);

    image_left.release(); image_right.release();
	
	//write vrpn position and quaternion to file
	for (int i=0;i<3;i++){
	    outfile << myPos(i) << " ";
	}

	for (int i=0;i<4;i++){
	    outfile << myQuat(i) << " ";
	}
	outfile << "\n";
	
	
	num_ima++;
	count++;
	
    }


  printf( "\nFinished grabbing images\n" );
  outfile.close();
  
  return 0;
}
