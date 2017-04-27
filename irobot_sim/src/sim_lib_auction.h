#ifndef __SIM_LIB_AUCTION_H____
#define __SIM_LIB_AUCTION_H____

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
#include <std_msgs/Bool.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <map>

#define NUM_WPT 13
#define NUM_TARGET 8

Eigen::MatrixXf Targets(NUM_TARGET, 3);

int wpt_request = 0;
int wpt_done = 0;
int robot_wpt_count = 0;

int raw_input = 1;

ros::Publisher auction_choice_pub;
ros::Publisher current_choice_pub;
ros::Subscriber peer_auction_choice_sub;
ros::Subscriber peer_current_choice_sub;
ros::Publisher auction_req_pub;
ros::Subscriber peer_auction_req_sub;
ros::Publisher auction_response_pub;
ros::Subscriber peer_response_sub;

bool receive_peer_choice = false;
bool flag_beat_in_transit = false;
Eigen::VectorXf peer_auction_choice(1);
std_msgs::Float32MultiArray auction_choice_msg;
std_msgs::Float32MultiArray current_choice_msg;

geometry_msgs::Point goal_msg;
geometry_msgs::Point target_msg;

std::map<int, Eigen::MatrixXf> EntryList;
std::map< int, std::map<int, Eigen::MatrixXf> > WaypointList;

int flag_getCurrentPosition;

// ***************************** Auction data *********************************** //
std_msgs::Float32MultiArray robot_record;
Eigen::VectorXf my_visit_history(NUM_TARGET+1); // Every entry has value from 0  to 1. 0 means never choose this target, 1 means this target has been finished.
					      // Note: The bid put on this target cannot exceed 1, since bid =  1/(1+distance);
Eigen::VectorXf peer_visit_history(NUM_TARGET+1), last_peer_history(NUM_TARGET+1);
bool available_target_list[NUM_TARGET];
int robot_working_state = 0; // 1 = transit, 2 = busy, 3 = done;

bool current_target_done = true;
bool receive_peer_response = false;

bool no_work_for_me = false;
bool no_work_for_peer = false;

Eigen::VectorXf CamPos(3), ocam_hand(3), ohand_world(3), ocam_world(3), q_h(4), q_cam(4);
Eigen::Matrix3f Rot_h2w, Rot_c2h, Rot_c2w;

struct NearestEntry{
  int which_target;
  int which_entry;
  float distance;
  float bid;
};

//*************************** Global supervised data ***************************//

NearestEntry my_chosen_entry;

Eigen::MatrixXf current_waypointlist;
Eigen::VectorXf last_waypoint_in_current_list(2);

Eigen::VectorXf waypoint(6);
Eigen::Vector3f target;

std::ofstream outfile_sim("/home/dukerama/hydro_catkin/src/irobot_sim/include/positions/Brain5_SimPoses.txt");
std::ofstream outfile_cam("/home/dukerama/hydro_catkin/src/irobot_sim/include/positions/Brain5_CamPoses.txt");
std::ofstream outfile_hand("/home/dukerama/hydro_catkin/src/irobot_sim/include/positions/Brain5_HandPoses.txt");
std::ofstream outfile_left("/home/dukerama/hydro_catkin/src/irobot_sim/include/detector/Brain5_cords_left.txt");
std::ofstream outfile_right("/home/dukerama/hydro_catkin/src/irobot_sim/include/detector/Brain5_cords_right.txt");

class rama_commander
{
  public:
    rama_commander();
    // Auction
    void loadAuctionTargets();
    void loadAuctionEntryList();
    void loadAuctionWaypointList();
    void showEntryList();
    void showWaypointList(int index_target, int index_entry);

    NearestEntry getAuctionNearestEntry();
    void sendAuctionWaypoint();
    void nextAuctionWaypoint();
    bool checkAllTargetDone();

    // New Auction
    void proposeAuction();
    void doAuction_v2();
    void peerAuctionChoiceCallback(const std_msgs::Float32MultiArray::ConstPtr& array);
    void peerCurrentChoiceCallback(const std_msgs::Float32MultiArray::ConstPtr& array);
    void peerAuctionReqCallback(const std_msgs::Float32MultiArray::ConstPtr& array);
    void peerAuctionResponseCallback(const std_msgs::Float32MultiArray::ConstPtr& array);
    void NoticePeerAuctionChoice();

    // For simulation
    void generateAuctionPosition();
    void generateWorkingPosition();
    void recordSimPosition();

  private:  
    int wpt_count; 
    int goal_target;
    int fx;
    int fy;
    // For Auction
    int which_waypoint_in_current_list;
    int ping_count;

    // For simulation
    int sim_count;
    double sim_time;
    double start_time;
    double duration;
    bool fresh_start;
    int auction_response_counter;
};

int getNumberofLine(std::string filename);
void waitForEnter();

#endif


