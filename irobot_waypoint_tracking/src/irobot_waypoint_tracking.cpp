// Receive commands from irobot_waypoint_commander;
// Realize commands by sending request to pid_server, servo_server and pgrcamera_server.
// Created by Yan Zhang, on Sep 11st, 2015.

#include "wpstracking_lib.cpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "irobot_waypoint_tracking");
  ros::NodeHandle n; 

  wps_sub = n.subscribe<geometry_msgs::Point>("waypoints", 1, WpsCallback);
  target_sub = n.subscribe<geometry_msgs::Point>("goal_target", 1, TargetCallback);
  wps_rqst_pub = n.advertise<geometry_msgs::Point>("wps_rqst", 1);
  all_sent_sub = n.subscribe<geometry_msgs::Point>("allsent", 1, AllSentCallback);

  printf("Robot : Hi! Please press 'any key + enter' to start the waypoint tracking...\n");
  waitForEnter();

  wps_count = 0;
  all_sent = 0;
  wps_done = 0; 

  // MAIN LOOP
  while( ros::ok() && all_sent==0 ) // Check if all waypoints are sent.
  {
    // Robot send waypoints requests to commander;
    wps_heard = 0; // Robot: I heard nothing now.
    tgt_heard = 0; 
    while((wps_heard==0) || (tgt_heard==0)){
      SendRequest();
      ros::spinOnce();
      if (all_sent == 1){break;}
    }
    SendiHeardyou();

    // Clients send request to PID_server, servo_server, pgrcamera_server to accomplish commands;

    printf("Robot : Doing job..."); // Test, the robot action detail is omitted.
    waitForEnter(); // Test, press any key + enter to mimic this waypoint work has been done.

    // Robot record its actual pose
  

    // Tell one waypoint work finished!
    SendWpsDone();
    ros::spinOnce();

    wps_done = 0;   

    wps_count++;
  }

  printf("Robot : All %d waypoints are done!\n", wps_count);

  return 0;
}
