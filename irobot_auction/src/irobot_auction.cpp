#include "lib_auction.cpp"

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
  ros::init(argc, argv, "Brain5_auction");
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
  
  ros::Rate loop_rate(10);

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

  std::string temp, result;

  rama_commander commander;
//  rama_robot robot;
  commander.loadAuctionTargets();
  commander.loadAuctionEntryList();
  commander.loadAuctionWaypointList();

  commander.showEntryList();
  commander.showWaypointList(3, 11);

  printf("Robot: Hi, I am %s ...\n", robotname.c_str());

  result = robotname + "/auction_choice";
  auction_choice_pub = n.advertise<std_msgs::Float32MultiArray>(result.c_str(), 1);
  result = robotname + "/current_choice";
  current_choice_pub = n.advertise<std_msgs::Float32MultiArray>(result.c_str(), 1);
  result = robotname + "/auction_req";
  auction_req_pub = n.advertise<std_msgs::Float32MultiArray>(result.c_str(), 1);
  result = robotname + "/auction_response";
  auction_response_pub = n.advertise<std_msgs::Float32MultiArray>(result.c_str(), 1);
  peer_auction_choice_sub = n.subscribe("Brain4/auction_choice", 1, &rama_commander::peerAuctionChoiceCallback, &commander);
  peer_current_choice_sub = n.subscribe("Brain4/current_choice", 1, &rama_commander::peerCurrentChoiceCallback, &commander);
  peer_auction_req_sub = n.subscribe("Brain4/auction_req", 1, &rama_commander::peerAuctionReqCallback, &commander);
  peer_response_sub = n.subscribe("Brain4/auction_response", 1, &rama_commander::peerAuctionResponseCallback, &commander);

  std::string publishname;
  robot_vrpn_sub = n.subscribe("Brain5Camera/pose", 1, vrpnCallback);
  publishname = robotname + "/cmd_vel";
  base_pub = n.advertise<geometry_msgs::Twist>(publishname.c_str(), 1);

  // Clear visit_history
  my_visit_history.setZero();
  peer_visit_history.setZero();

  int iii;
  for (iii = 0; iii < NUM_TARGET; iii++)
  {
    available_target_list[iii] = true;
  }

  if (raw_input == 1) // Choose raw image calibration results
  {
      Rot_c2h << -0.2113,  0.0064, -0.9774,
                  0.9772, -0.0199, -0.2114,
                 -0.0208, -0.9998, -0.0020; 
 
      ocam_hand << -0.0655203, -0.0244237, 0.0046857;
      std::cout << "Using raw image calibration results..." << "\n" << std::endl;
  }
  else
  {
      Rot_c2h <<  0.0918,  0.0170,  0.9956,
                 -0.9921,  0.0869,  0.0900,
                 -0.0850, -0.9961,  0.0248; 
      ocam_hand << 0.0750, -0.0221, 0.0023;

      std::cout << "Using rectified image calibration results..." << "\n" << std::endl; 
  }

// Servo_Base calibration is done on Oct 14th. Redefine robots need recalibration.
  Rot_r2n <<  0.6470,  0.7625, 0,
	     -0.7625,  0.6470, 0,
    	      0,       0,      1;

  ohand2servo_hand << -0.02236, -0.02441, 0;

  sendServoSpeed();
  initialServoTarget();
  robot_wpt_count = 0;
  recordCamPose();

  current_target_done = true;
  flag_beat_in_transit = false; 
  robot_working_state = 1; // Set robot to "transit" state;

  waypoint_controller waypoint(n);

  waypoint.testVrpn();

  printf("Robot : Please press 'any key + enter' to start the waypoint tracking...\n");
  waitForEnter();

// Keep timing
  double auction_start, current_time;
  current_time = 0.0;
  recordCamPoseTime(current_time);

  while(ros::ok() && (robot_working_state != 3))
  {
    if (current_target_done)
    {
      commander.proposeAuction();
    }

    if (robot_working_state == 3) {break;}

    if (count == 601)
    { auction_start =  ros::Time::now().toSec(); }

    commander.sendAuctionWaypoint();

//  Robot is navigated to waypoints;
    if (robot_working_state == 1)
    {
      std::cout << "Commander: In transit, detect obstacles!" << std::endl;
      bool if_obs = detectObstacles();
      if (if_obs) { 
	collisionAvoidance();	
    	if (!flag_beat_in_transit) {
		calcOrder(); 
   		waypoint.goWaypoint(w_hat, Rg_hat);
    	} 	
      } else {
		calcOrder(); 
   		waypoint.goWaypoint(w_hat, Rg_hat);
        }
    } else {
	calcOrder(); 
   	waypoint.goWaypoint(w_hat, Rg_hat);
    }

//  Robot has got to the waypoint;

    if (!flag_beat_in_transit)
    {
      if (robot_working_state == 1)
      { 
          commander.NoticePeerAuctionChoice();
      }

      waitForServoStill();

//  take image here
      std::cout << "Waiting for Servo!" << std::endl;
      ros::Duration(1.0).sleep();

      std::cout << "Robot is taking images!" << " " << "flag_beat_in_transit: " << flag_beat_in_transit << std::endl;

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

//  Record Position here
      recordImages(count, image_left, image_right);

      image_left.release(); image_right.release();

      recordCamPose();

      current_time = ros::Time::now().toSec() - auction_start;
      recordCamPoseTime(current_time);

      count++;
      robot_wpt_count++;

      commander.nextAuctionWaypoint();
    }
    else
    {
      recordCamPose();

      current_time = ros::Time::now().toSec() - auction_start;
      recordCamPoseTime(current_time);      
    }
  }
  printf("Commander : Job finished!\n");
  outfile_cam.close();
  outfile_hand.close();
  outfile_handTime.close();
  outfile_target.close();
  return 0;
}

