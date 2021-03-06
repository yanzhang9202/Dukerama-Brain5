// This script shows the trajectory of iRobot or other rigid body tracked by VRPN, 
// Created by Yan Zhang, Aug 28th, 2015.

#include "track_visual.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "track_visual");
  ros::NodeHandle n;
  ros::Rate loop_rate(50);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/Brain5Track", 1);

  ros::Subscriber marker_sub = n.subscribe("/Brain5Camera/pose", 1, markerCallback);

  while (ros::ok())
  {
    ros::spinOnce();
    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    loop_rate.sleep();
  }
}

void markerCallback(const geometry_msgs::TransformStamped msg)
{
  points.header.frame_id = line_strip.header.frame_id = "/optitrak";
  points.header.stamp = line_strip.header.stamp = ros::Time::now();

  points.ns =  line_strip.ns = "Brain5";
  points.action = line_strip.action = visualization_msgs::Marker::ADD;

  points.id = 0;
  line_strip.id = 1;

  points.type = visualization_msgs::Marker::ARROW;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  points.scale.x = 0.2;
  points.scale.y = 0.02;
  points.scale.z = 0.02;

  line_strip.scale.x = 0.02;

  points.color.r = 1.0f;
  points.color.a = 1.0;

  line_strip.color.b = 1.0f;
  line_strip.color.a = 1.0;

  line_strip.pose.orientation.w = 1.0;
  
  p.x = msg.transform.translation.x;
  p.y = msg.transform.translation.y;
  p.z = msg.transform.translation.z;
  line_strip.points.push_back(p);

  points.pose.position.x = msg.transform.translation.x;
  points.pose.position.y = msg.transform.translation.y;
  points.pose.position.z = msg.transform.translation.z; 

  points.pose.orientation.x =msg.transform.rotation.x;
  points.pose.orientation.y =msg.transform.rotation.y;
  points.pose.orientation.z =msg.transform.rotation.z;
  points.pose.orientation.w =msg.transform.rotation.w;
}



