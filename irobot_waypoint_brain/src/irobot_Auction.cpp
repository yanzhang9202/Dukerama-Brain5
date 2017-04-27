#include "ramabot_brain.cpp"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pgrcamera_driver/TakePicAction.h>
#include <boost/bind.hpp>

#include "FlyCapture2.h"

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

  std::string robotname, temp, result;
  n.getParam("/robot/name", robotname);

//  cv::namedWindow("right",CV_WINDOW_NORMAL);
//  cv::namedWindow("left",CV_WINDOW_NORMAL);

//  result = robotname+"/take_pic";
//  actionlib::SimpleActionClient<pgrcamera_driver::TakePicAction> ac(result.c_str(), true); //First argument in ac constructor is the server to connect to

//  cv::Mat image_left,image_right;

//  image_transport::ImageTransport it(n);

//  result = robotname+"/left/image_rect_color";
//  image_transport::Subscriber pic_left=it.subscribe(result.c_str(),1,boost::bind(convert_image,_1,&image_left));
//  result = robotname+"/right/image_rect_color";
//  image_transport::Subscriber pic_right=it.subscribe(result.c_str(),1,boost::bind(convert_image,_1,&image_right));

//  pgrcamera_driver::TakePicGoal goal;
//  goal.acquire=true;

//  bool updated=false;
//  int count=601;
//	
//  std::cout<<"\n Initiating Camera... \n";

//  ros::Rate loop_rate(60);

//  //ping camera driver and wait for it to start
//  while (ros::ok() && !updated)
//  {
//	ac.waitForServer();
//	ac.sendGoal(goal);

//	if (!image_left.empty() && !image_right.empty())
//	{
//		imshow("right",image_right);
//		imshow("left",image_left);
//		cv::waitKey(30);
//		updated = true;

//	}

//	ros::spinOnce();
//	loop_rate.sleep();
//  }

  rama_commander commander;
  rama_robot robot;
  commander.loadTargets();
  commander.loadEntryList();
  commander.loadAuctionWaypointList();

//  showWaypointList(1, 1);
  printf("Robot : Hi, I am %s ...\n", robotname.c_str());

//  temp = "/state"; result = robotname + temp; 
//  robot_state_pub = n.advertise<geometry_msgs::Point>(result.c_str(), 1); // "robot/state"
//  temp = "/target"; result = robotname + temp; 
//  robot_target_pub = n.advertise<geometry_msgs::Point>(result.c_str(), 1);
//  temp = "/waypoint"; result = robotname + temp; 
//  waypoint_pub = n.advertise<geometry_msgs::Point>(result.c_str(), 1);
//  temp = "/pid_state"; result = robotname + temp; 
//  pid_state_sub = n.subscribe<geometry_msgs::Point>(result.c_str(), 1, pidCallback);

  ros::Subscriber cam_pose_sub = n.subscribe("/Brain5Camera/pose", 1, recordCamPoseCallback);

  result = robotname + "/record";
  robot_record_pub = n.advertise<std_msgs::Float32MultiArray>(result.c_str(), 1);
  result = "Brain4" + "/record";
  peer_record_sub = n.subscribe(result.c_str(), 1, peerRecordCallback); // Define callback in cpp!!!!!!!!!!!!!

  printf("Robot : Please press 'any key + enter' to start the waypoint tracking...\n");
  robot.waitForEnter();

  int robot_wpt_count = 0;
  no_waypoints = 1;

  // Clear visit_history
  visit_history.setZero();
  peer_visit_history.setZero();
  next_waypoint_set = false;
//  auction_done = false; // Everytime set auction_done to 0, two robots will choose nearest target and auction.
  current_target_done = true;

  while(ros::ok() && commander.cluster_check_all_sent() == 0)
  {
    if (current_target_done)
    {
      if (next_waypoint_set)
      { command.Next2CurrentWaypoints(); }
      else
      {
        propose_auction = true; reach_agreement = false; // Here is start of the mission or finish of the last mission.
        while (propose_auction && ~reach_agreement)
        {
          command.chooseAuctionWaypointList(propose_auction); // Here find the nearest target, calculate bids, publish to peers, peers publish back, reach agreement.
        }
      }
    }
    else
    {
      command.getAuctionNextWaypoints();
    }

    robot.adjustOrientation(); // There is a while() in this function, and ros::spinOnce in while(); During this function, Brain5 subscribes to Brain4's visit_history.
                               // The visit history callback function calculate's next entry and target, put bids, publish to Brain5 and set next waypoint for itself.

    // take image here
//    ac.sendGoal(goal);
//    ac.waitForResult();
//    ros::spinOnce();
//    ros::spinOnce();
//    ros::spinOnce();
//    ac.sendGoal(goal);
//    ac.waitForResult();
//    ros::spinOnce();
//    ros::spinOnce();
//    ros::spinOnce();
//    ac.sendGoal(goal);
//    ac.waitForResult();
//    ros::spinOnce();
//    ros::spinOnce();
//    ros::spinOnce();
//    robot.recordImages(count, image_left, image_right);

//    robot.recordCamPose();

//    count++;

  }

  printf("Commander : Job finished!\n");
  return 0;

}

//    if (ClusterDP_fresh_start == 1)  // it's the fresh start of the mission
//    {  chosen_entry = commander.chooseNearestEntry();
//       commander.getClusterWaypoints(chosen_entry);
//       ClusterDP_fresh_start = 0;  }
    // Else, (1) Commander send next cluster waypoint and target position 
    //       (2) Robot publishes next waypoint and target 
    //       (3) Robot goes to next waypoint  
    //       (4) Robot takes images and record current positions

//    robot.rqstWaypoints();
//    commander.getClusterWaypoints();

//    robot.adjustOrientation();

//    printf("Press any key + Enter to take images...\n");
//    robot.waitForEnter();

//    //take picture
//    ac.sendGoal(goal);
//    ac.waitForResult();
//    ros::spinOnce();
//    ros::spinOnce();
//    ros::spinOnce();
//    ac.sendGoal(goal);
//    ac.waitForResult();
//    ros::spinOnce();
//    ros::spinOnce();
//    ros::spinOnce();
//    ac.sendGoal(goal);
//    ac.waitForResult();
//    ros::spinOnce();
//    ros::spinOnce();
//    ros::spinOnce();
//    loop_rate.sleep();

//    robot.recordImages(count, image_left, image_right);

//    robot.recordCamPose();

//    count++;

//    commander.nextWaypoints();
//    robot_wpt_count++;      




//  cv::namedWindow("right",CV_WINDOW_NORMAL);
//  cv::namedWindow("left",CV_WINDOW_NORMAL);

//  actionlib::SimpleActionClient<pgrcamera_driver::TakePicAction> ac("take_pic", true); //First argument in ac constructor is the server to connect to

//  cv::Mat image_left,image_right;

//  image_transport::ImageTransport it(n);
//  image_transport::Subscriber pic_left=it.subscribe("/stereo/left/image_rect_color",1,boost::bind(convert_image,_1,&image_left));
//  image_transport::Subscriber pic_right=it.subscribe("/stereo/right/image_rect_color",1,boost::bind(convert_image,_1,&image_right));

//  pgrcamera_driver::TakePicGoal goal;
//  goal.acquire=true;
//  
//  ros::Rate loop_rate(60);

//  bool updated=false;
//  int count=601;
//	
//  std::cout<<"\n Initiating Camera... \n";

//  //ping camera driver and wait for it to start
//  while (ros::ok() && !updated)
//  {
//	ac.waitForServer();
//	ac.sendGoal(goal);

//	if (!image_left.empty() && !image_right.empty())
//	{
//		imshow("right",image_right);
//		imshow("left",image_left);
//		cv::waitKey(30);
//		updated = true;

//	}

//	ros::spinOnce();
//	loop_rate.sleep();
//  }

//  rama_commander commander;
//  rama_robot robot;

//  commander.loadWaypoints();
//  commander.loadTargets();

//  robot_state_pub = n.advertise<geometry_msgs::Point>("robot/state", 1);
//  robot_target_pub = n.advertise<geometry_msgs::Point>("robot/target", 1);

//  waypoint_pub = n.advertise<geometry_msgs::Point>("robot/waypoint", 1);
//  
//  pid_state_sub = n.subscribe<geometry_msgs::Point>("robot/pid_state", 1, pidCallback);

//  int robot_wpt_count = 0;

//  printf("Robot : Hi! Please press 'any key + enter' to start the waypoint tracking...\n");
//  robot.waitForEnter();

//  ros::Subscriber cam_pose_sub = n.subscribe("/Brain5Camera/pose", 1, recordCamPoseCallback);

//  // MAIN LOOP
//  while( ros::ok() && (commander.check_all_sent()==0) ) // Check if all waypoints are sent.
//  {
//    robot.rqstWaypoints();

//    commander.sendWaypoints();

//  // Tell Servo node to start, adjust base and servo position (To be finished)

//    robot.adjustOrientation();

//    //  Robot should take images here and record its hand position from VRPN; //

//    printf("Press any key + Enter to take images...\n");
//    robot.waitForEnter();

//    //take picture
//    ac.sendGoal(goal);
//    ac.waitForResult();
//    ros::spinOnce();
//    ros::spinOnce();
//    ros::spinOnce();
//    ac.sendGoal(goal);
//    ac.waitForResult();
//    ros::spinOnce();
//    ros::spinOnce();
//    ros::spinOnce();
//    ac.sendGoal(goal);
//    ac.waitForResult();
//    ros::spinOnce();
//    ros::spinOnce();
//    ros::spinOnce();
//    loop_rate.sleep();

//    robot.recordImages(count, image_left, image_right);

////    robot.detectCircles(count, image_left, image_right);

//    robot.recordCamPose();

//    count++;

//    // ----------------------------To be finished-----------------------------//

//    commander.nextWaypoints();
//    robot_wpt_count++;
//  }

//  printf("Commander : Job finished!\n");
//  outfile_cam.close();
//  outfile_hand.close();
//  outfile_left.close();
//  outfile_right.close();
