// Waypoints_commander publishes waypoints and initial guess of target positions, subscribes to irobot job state.
// Created by Yan Zhang, on Sep 11st, 2015.

#include "wpslib.cpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "irobot_waypoint_commander");
  ros::NodeHandle n; 

  wps_pub = n.advertise<geometry_msgs::Point>("waypoints", 1);
  target_pub = n.advertise<geometry_msgs::Point>("goal_target", 1);
  wps_rqst_sub = n.subscribe<geometry_msgs::Point>("wps_rqst", 1, WpsRqstCallback);
  all_sent_pub = n.advertise<geometry_msgs::Point>("allsent", 1);

  loadWaypoints();
  loadTargets();

  // MAIN LOOP
  while( ros::ok() && wps_count<NUM_WPS)
  {
    ros::spin(); // Keep listening to "wps_rqst"
  }

  return 0;
}

