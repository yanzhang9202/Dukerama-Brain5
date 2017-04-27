// Restore the Optitrack_VRPN frame as the fixed world frame
// Finally, it turns out this script is useless. The /world frame is the same as the returned /optitrak frame. But this is how we get the fixed world frame.
// Created by Yan Zhang, on Aug 28th, 2015

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

void poseCallback(const geometry_msgs::TransformStamped msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z) );
  transform.setRotation(tf::Quaternion(msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/Brain4Camera"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_world_broadcaster");
  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("/Brain4Camera/pose", 1, &poseCallback);

  ros::spin();
  return 0;
};
