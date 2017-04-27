#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pgrcamera_driver/TakePicAction.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <iostream>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "receive_pic");
  ros::NodeHandle n;

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<pgrcamera_driver::TakePicAction> ac("Brain5/take_pic", true); //First argument in ac constructor is the server to connect to

  ros::Rate loop_rate(50);

  while (ros::ok()) {
    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time
    
    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    pgrcamera_driver::TakePicGoal goal;
    goal.acquire=true;

    //int test;
    //std::cin >> test;

    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
      {
	actionlib::SimpleClientGoalState state = ac.getState();
	ROS_INFO("Action finished: %s",state.toString().c_str());
      }
    else
      ROS_INFO("Action did not finish before the time out.");

    //result=ac.getResult();

    ros::spinOnce();
    loop_rate.sleep();
  }
  //exit
  return 0;
}
