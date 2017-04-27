#include "ramabot_brain.cpp"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pgrcamera_driver/TakePicAction.h>
#include <boost/bind.hpp>

#include "FlyCapture2.h"

#include "waypoint_controller.cpp"

using namespace FlyCapture2;
using namespace cv;

#define SERIAL_LEFT 13200942
#define SERIAL_RIGHT 13200944

void convert_image(const sensor_msgs::ImageConstPtr& msg,cv::Mat* image){
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "irobot_waypoint_brain");
  ros::NodeHandle n;

  cv::namedWindow("right",CV_WINDOW_NORMAL);
  cv::namedWindow("left",CV_WINDOW_NORMAL);

  std::string robotname, topicname;
  n.getParam("/robot/name", robotname); topicname = robotname+"/take_pic";
  actionlib::SimpleActionClient<pgrcamera_driver::TakePicAction> ac(topicname.c_str(), true); //First argument in ac constructor is the server to connect to

  cv::Mat image_left,image_right;

  image_transport::ImageTransport it(n);
  topicname = robotname+"/left/image_rect_color";
  image_transport::Subscriber pic_left=it.subscribe(topicname.c_str(),1,boost::bind(convert_image,_1,&image_left));
  topicname = robotname+"/right/image_rect_color";
  image_transport::Subscriber pic_right=it.subscribe(topicname.c_str(),1,boost::bind(convert_image,_1,&image_right));

  pgrcamera_driver::TakePicGoal goal;
  goal.acquire=true;
  
  ros::Rate loop_rate(60);

  bool updated=false;
  int count=601;
	
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

  rama_commander commander;
  rama_robot robot;

  commander.loadWaypoints();
  commander.loadTargets();

  topicname = robotname+"/state";
  robot_state_pub = n.advertise<geometry_msgs::Point>(topicname.c_str(), 1);
  topicname = robotname+"/target";
  robot_target_pub = n.advertise<geometry_msgs::Point>(topicname.c_str(), 1);
  topicname = robotname+"/waypoint";
  waypoint_pub = n.advertise<geometry_msgs::Point>(topicname.c_str(), 1);
  topicname = robotname+"/pid_state";
  pid_state_sub = n.subscribe<geometry_msgs::Point>(topicname.c_str(), 1, pidCallback);

  int robot_wpt_count = 0;
  printf("Robot : Hi, I am %s ...\n", robotname.c_str());

  printf("Robot : Please press 'any key + enter' to start the waypoint tracking...\n");
  robot.waitForEnter();

  ros::Subscriber cam_pose_sub = n.subscribe("/Brain5Camera/pose", 1, recordCamPoseCallback);

    if (raw_input == 1)
    {
      Rot_c2h << -0.2113,  0.0064, -0.9774,
                  0.9772, -0.0199, -0.2114,
                 -0.0208, -0.9998, -0.0020; 
      ocam_hand << -0.0655203, -0.0244237, 0.0046857;
    }
    else
    {
//   Brain5 Calibration on Oct 30th
      Rot_c2h <<  0.0918,  0.0170,  0.9956,
                 -0.9921,  0.0869,  0.0900,
                 -0.0850, -0.9961,  0.0248; 
      ocam_hand << 0.0750, -0.0221, 0.0023;
    }

  waypoint_controller waypoint(n);

  waypoint.testVrpn();

  // MAIN LOOP
  while( ros::ok() && (commander.check_all_sent()==0) ) // Check if all waypoints are sent.
  {
    robot.rqstWaypoints();

    commander.sendWaypoints();

  // Tell Servo node to start, adjust base and servo position (To be finished)

//    robot.adjustOrientation();

    calcOrder(cam_goal, tgt);
    waypoint.goWaypoint(w_hat, Rg_hat);

    //  Robot should take images here and record its hand position from VRPN; //

    printf("Press any key + Enter to take images...\n");
    robot.waitForEnter();

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

    robot.localrecordImages(count, image_left, image_right);

//    robot.detectCircles(count, image_left, image_right);

    image_left.release(); image_right.release();

    robot.recordCamPose();

    count++;

    // ----------------------------To be finished-----------------------------//

    commander.nextWaypoints();

    robot_wpt_count++;
  }

  printf("Commander : Job finished!\n");
  outfile_cam.close();
  outfile_hand.close();
  outfile_left.close();
  outfile_right.close();

  return 0;
}

