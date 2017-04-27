#ifndef __WPSTRACKING_LIB_cpp
#define __WPSTRACKING_LIB_cpp

#include "wpstracking_lib.h"

void waitForEnter(){
	char Enter;
	std::cin >> Enter;
}

void WpsCallback(const geometry_msgs::Point msg){
  // Store waypoint message

  // I heard you
  wps_heard = 1;
}

void TargetCallback(const geometry_msgs::Point msg){
  // Store target message

  // I heard you
  tgt_heard = 1;
}

void AllSentCallback(const geometry_msgs::Point msg){
  printf("Robot : Commander tells me all the waypoints are sent!\n");
  all_sent = 1;
}

void SendRequest(){
  wps_request = 1;
  wps_done = 0;
  rqst_msg.x = wps_request;
  rqst_msg.y = wps_done;
  wps_rqst_pub.publish(rqst_msg);
}

void SendiHeardyou(){
  wps_request = 0;
  wps_done = 0;
  rqst_msg.x = wps_request;
  rqst_msg.y = wps_done;
  wps_rqst_pub.publish(rqst_msg);
}

void SendWpsDone(){
  wps_request = 0;
  wps_done = 1;
  rqst_msg.x = wps_request;
  rqst_msg.y = wps_done;
  wps_rqst_pub.publish(rqst_msg);
}

#endif
