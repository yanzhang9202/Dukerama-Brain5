#ifndef __RAMABOT_BRAIN_H____
#define __RAMABOT_BRAIN_H____

#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include <string>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <map>

#define NUM_WPT 13
#define NUM_TARGET 1

Eigen::MatrixXf Horizon(NUM_WPT, 3);
Eigen::MatrixXf Targets(NUM_TARGET, 3);

int wpt_request = 0;
int wpt_done = 0;
int robot_wpt_count = 0;

int raw_input = 0;

Eigen::Vector3f waypoint;
Eigen::Vector3f target;

ros::Publisher robot_state_pub;
ros::Publisher robot_target_pub;
ros::Publisher waypoint_pub;
ros::Publisher robot_record_pub;

ros::Subscriber pid_state_sub;
ros::Subscriber peer_record_sub;

geometry_msgs::Point goal_msg;
geometry_msgs::Point target_msg;

Eigen::VectorXf CamPos(3), ocam_hand(3), ohand_world(3), ocam_world(3), q_h(4), q_cam(4);
Eigen::Matrix3f Rot_h2w, Rot_c2h, Rot_c2w;

std::ofstream outfile_cam("/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/positions/CamPoses.txt");
std::ofstream outfile_hand("/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/positions/HandPoses.txt");
std::ofstream outfile_left("/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/detector/cords_left.txt");
std::ofstream outfile_right("/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/detector/cords_right.txt");

// ***************************** Cluster DP data ******************************* //

std::map<int, Eigen::MatrixXf> EntryList;
std::map< int, std::map<int, Eigen::MatrixXf> > WaypointList;
Eigen::MatrixXf chosen_waypoints;
int no_waypoints = 0;
int flag_getCurrentPosition;

std::ofstream outfile_entry("/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/positions/WhichWPTList.txt");

// ***************************** Auction data *********************************** //
std_msgs::Float32MultiArray robot_record;
Eigen::VectorXf my_visit_history(NUM_TARGET); // Every entry has value from 0  to 1. 0 means never choose this target, 1 means this target has been finished.
					   // Note: The bid put on this target cannot exceed 1, since bid =  1/(1+distance);
Eigen::VectorXf peer_visit_history(NUM_TARGET);
//bool auction_done = false;
bool propose_auction = false;
bool reach_agreement = false;
bool next_waypoints_set = false;
bool current_target_done = true;
bool receive_peer_response = false;

struct NearestEntry{
  int which_target;
  int which_entry;
  float distance;
  float bid;
};

class rama_commander
{
  public:
    rama_commander();

    void loadWaypoints();
    void loadTargets();
    Eigen::Vector3f sendWaypoints();
    int check_all_sent();
    void nextWaypoints();
    cv::Mat downsample(cv::Mat image);
    cv::Mat findCircle(cv::Mat image);
    // Cluster DP
    void loadClusterTargets();
    void loadEntryList();
    void loadClusterWaypointList();
    Eigen::VectorXi chooseNearestEntry();
    void getClusterWaypoints();
    int cluster_check_all_sent();
    // Auction
    void loadAuctionWaypointList();
    void chooseAuctionWaypointList(bool propose_auction);
    NearestEntry getAuctionNearestEntry();
    void peerRecordCallback(const std_msgs::Float32MultiArray::ConstPtr& array);
  private:  
    int wpt_count; 
    int goal_target;
    int fx;
    int fy;
};

class rama_robot
{
  public:
    rama_robot();

    void waitForEnter();
    void rqstWaypoints();
    void sendWptDone();
    void startServo();
    void goingWaypoint();
    void localrecordImages(int count, cv::Mat image_left, cv::Mat image_right);
    void recordImages(int count, cv::Mat image_left, cv::Mat image_right);
    void detectCircles(int count, cv::Mat image_left, cv::Mat image_right);
    void recordCamPose();
    void adjustOrientation();
  private:
    int robot_wpt_count;
};

int pid_state = 0;
int ori_pid_state = 0;
int straight_pid_state = 0;
void pidCallback(const geometry_msgs::Point msg);
void recordCamPoseCallback(const geometry_msgs::TransformStamped msg);
int getNumberofLine(std::string filename);

// Cluster DP
void showEntryList();
void showWaypointList(int index_target, int index_entry);

// Auction
void peerRecordCallback(const std_msgs::Float32MultiArray::ConstPtr& array);

// New controller function and variables
void calcOrder(Eigen::Vector3f cam, Eigen::Vector3f targ);
Eigen::Matrix3f buildCamGoal(Eigen::Vector3f az);
Eigen::Matrix2f matrix2d(int index1, int index2, Eigen::Matrix3f mat);

Eigen::Vector3f cam_goal, hand_goal, tgt;
Eigen::Matrix3f Rcam_goal, Rhand_goal;
Eigen::Vector2f w_hat;
Eigen::Matrix2f Rg_hat;

#endif
