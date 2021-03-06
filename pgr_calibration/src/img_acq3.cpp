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
#include <geometry_msgs/Twist.h>


#include "FlyCapture2.h"

#define SERIAL_LEFT 13200942
#define SERIAL_RIGHT 13200944

geometry_msgs::Twist transl_msg;
float linMove[25]={ .05, 0, 0, .05, 0, 0, .05, 0, 0, .05, 0, 0, .05, 0, 0, .05, 0, 0, .05, 0, 0, .2, 0, 0, .2};
float angRott[25]={ 0, .3, -.5, .3, .3, -.5, .3, .3, -.5, .3, .3, -.5, .3, .3, -.5, .3, .3, -.5, .3, .3, -.5, .3, .3, -.5, 0};

using namespace FlyCapture2;
using namespace cv;


void waitFor(unsigned int secs){
	unsigned int retTime;
	retTime = time(0) + secs;
	while (time(0) < retTime);
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
  cv::namedWindow("left",CV_WINDOW_NORMAL);
  cv::namedWindow("right",CV_WINDOW_NORMAL);
  

//publisher to the turtlebot creates
//commented out
//ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);
 // std::ofstream outfile("/home/dukerama/catkin_ws/src/pgr_calibration/include/positions/Positions.txt");

  actionlib::SimpleActionClient<pgrcamera_driver::TakePicAction> ac("take_pic", true); //First argument in ac constructor is the server to connect to

  cv::Mat image_left,image_right;

  std::string myself="Brain5Camera";
 // CameraTrackable* me=new CameraTrackable();

  //vrpnNode vrpn(n,me,myself);
  image_transport::ImageTransport it(n);
  image_transport::Subscriber pic_left=it.subscribe("/stereo/left/image_raw",1,boost::bind(convert_image,_1,&image_left));
  image_transport::Subscriber pic_right=it.subscribe("/stereo/right/image_rect_color",1,boost::bind(convert_image,_1,&image_right));

//code to write raw images in addition to rectified ones :))


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

  //Eigen::Vector3d myPos, myLastPos;
 // Eigen::Matrix3d myRot, myLastRot;
  while (ros::ok() && num_ima<20 /*&& exit%256!=27/* && exit!=/*27*/)
    {


//robot moving code goes here (additional code for handeye, to move the 'grip')






      ros::spinOnce();
      //std::cout << "Exit is: " << exit << std::endl;
      // wait for the action server to start       



                                                                                                                                                 
      if (!updated){
	ac.waitForServer(); //will wait for infinite time
	// send a goal to the action
	ac.sendGoal(goal);
	//wait for the action to return                                                                                                                                                                    
	bool finished_before_timeout = ac.waitForResult(ros::Duration(0.0));
      }
      //std::cout << exit << std::endl;



     // myPos=me->getCameraPosition();
     // myRot=me->getCameraRotation();



      if (!image_left.empty() && !image_right.empty()){
	updated=true;
	/*imshow("left",image_left);
	  imshow("right",image_right);
	  exit=waitKey(30);
	  //if (exit%256==32){
	  while (ros::ok() && num_ima<25){*/
	imshow("left",image_left);
	imshow("right",image_right);
	exit=waitKey(30);
	now=ros::Time::now();
	
//comment out
	//string mystring;
	//mystring = "xy";
	//while (mystring!="y"){
	//std::cout<<"press y and Hit Enter Now";
	//std::cin >> mystring;
	//}

waitFor(5);

	ac.sendGoal(goal);
	ros::spinOnce();
	if (!started){
	  if((now-last).toSec()>10){
	    started=true;
	  }
	  continue;
	}


	//myLastPos=myPos;
	//myLastRot=myRot;
	//myPos=me->getCameraPosition();
	//myRot=me->getCameraRotation();
	    
	//SAVE POSITION AND ROTATION HERE
	    
	    
	//wait for the action to return                                                                                                     
	//bool finished_before_timeout = ac.waitForResult(ros::Duration(0.0));
	    
	// Create a unique filename
	char filename_left[512];
	sprintf( filename_left, "/home/dukerama/catkin_ws/src/pgr_calibration/include/calib_images/left%d.jpg", count );
	  
	char filename_right[512];
	sprintf( filename_right, "/home/dukerama/catkin_ws/src/pgr_calibration/include/calib_images/right%d.jpg", count );

	
	    
	// Save the image. If a file format is not passed in, then the file
	// extension is parsed to attempt to determine the file format.
	    
	if ((now-last).toSec()>5){
	  //std::cout << "My position is: \n" << myLastPos << std::endl;
	 // last=now;
	  imwrite(filename_left,image_left);
	  imwrite(filename_right,image_right);
	  //std::cout << "Position time is: " << me->getTimeStamp() << std::endl;
	 // for (int i=0;i<3;i++){
	  //  outfile << myLastPos(i) << " ";
	 // }
	//  std::cout << "My rotation is: \n" << myLastRot << std::endl;   
	  //for (int i=0;i<3;i++){
	 //   for (int j=0;j<3;j++){
	  //    outfile << myLastRot(i,j) << " ";
	  //  }
	 // }
	 //     
	  //outfile << "\n";
	  
	  
	  printf("Count=%d %d %d %d %d %d %d %d\n"/*Press space to take a photo or escape to quit\n"*/, count,count,count,count,count,count,count,count);
	  count++;
	  num_ima++;
	// MOVE Robot Around
//transl_msg.linear.x=linMove[num_ima+1];
//transl_msg.angular.z=angRott[num_ima+1];
//chatter_pub.publish(transl_msg);
//ROS_INFO("I printed %f %f", transl_msg.linear.x, transl_msg.angular.z);
//waitFor(3);
	}
	//}	  
      }
	
      ros::spinOnce();
    }
  
  printf( "\nFinished grabbing images\n" );
  //outfile.close();
  
  return 0;
}
