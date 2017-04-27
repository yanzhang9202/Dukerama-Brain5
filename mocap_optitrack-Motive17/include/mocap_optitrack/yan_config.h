#ifndef __YAN_CONFIG_H__
#define __YAN_CONFIG_H__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "mocap_datapackets.h"

class PublishedMarker
{
  private:
  ros::NodeHandle n;

  std::string marker_pose_topic;
  //std::string parent_frame_id;
  //std::string child_frame_id;

  bool publish_marker_pose;

  ros::Publisher marker_pose_pub;

  bool validateParam(XmlRpc::XmlRpcValue &, const std::string &);

  public:
  PublishedMarker(XmlRpc::XmlRpcValue &);
  void publish(Marker &);
};

typedef std::map<int, PublishedMarker> MarkerMap;
typedef std::pair<int, PublishedMarker> MarkerItem;

#endif  // __YAN_CONFIG_H__
