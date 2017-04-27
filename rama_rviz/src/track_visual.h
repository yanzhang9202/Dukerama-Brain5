#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>

#include <cmath>

visualization_msgs::Marker points, line_strip;
geometry_msgs::Point p;

void markerCallback(const geometry_msgs::TransformStamped msg);

