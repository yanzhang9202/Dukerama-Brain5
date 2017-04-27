#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include "mocap_optitrack/yan_config.h"

const std::string MARKER_POSE_TOPIC_PARAM_NAME = "pose";

PublishedMarker::PublishedMarker(XmlRpc::XmlRpcValue &config_node)
{
  // load configuration for this rigid body from ROS
  publish_marker_pose = validateParam(config_node, MARKER_POSE_TOPIC_PARAM_NAME);

  if (publish_marker_pose)
  {
    marker_pose_topic = (std::string&) config_node[MARKER_POSE_TOPIC_PARAM_NAME];
    marker_pose_pub = n.advertise<geometry_msgs::PointStamped>(marker_pose_topic, 1000);
  }	
}

void PublishedMarker::publish(Marker &marker)
{
  // don't do anything if no new data was provided
  if (!marker.has_data())
  {
    return;
  }
  // NaN?
  if (marker.positionX != marker.positionX)
  {
    return;
  }

  // TODO Below was const, see if there a way to keep it like that.
  geometry_msgs::PointStamped point = marker.get_ros_pose();

  if (publish_marker_pose)
  {
    //point.header.frame_id = parent_frame_id;
    marker_pose_pub.publish(point);
  }

}

bool PublishedMarker::validateParam(XmlRpc::XmlRpcValue &config_node, const std::string &name)
{
  if (!config_node.hasMember(name))
  {
    return false;
  }

  if (config_node[name].getType() != XmlRpc::XmlRpcValue::TypeString)
  {
    return false;
  }

  return true;
}
