#ifndef __LIB_AUCTION_H____
#define __LIB_AUCTION_H____

#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Bool.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include "lib_robot.cpp"

#include <map>

#define NUM_WPT 13
#define NUM_TARGET 8

Eigen::MatrixXf Targets(NUM_TARGET, 3);

int wpt_request = 0;
int wpt_done = 0;

ros::Publisher auction_choice_pub;
ros::Publisher current_choice_pub;
ros::Subscriber peer_auction_choice_sub;
ros::Subscriber peer_current_choice_sub;
ros::Publisher auction_req_pub;
ros::Subscriber peer_auction_req_sub;
ros::Publisher auction_response_pub;
ros::Subscriber peer_response_sub;

bool receive_peer_choice = false;
Eigen::VectorXf peer_auction_choice(1);
std_msgs::Float32MultiArray auction_choice_msg;
std_msgs::Float32MultiArray current_choice_msg;

geometry_msgs::Point goal_msg;
geometry_msgs::Point target_msg;

std::map<int, Eigen::MatrixXf> EntryList;
std::map< int, std::map<int, Eigen::MatrixXf> > WaypointList;

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

void recordImages(int count, cv::Mat image_left, cv::Mat image_right);
void recordCamPose();
void recordCamPoseTime(double current_time);

bool detectObstacles();
Eigen::VectorXf obstacle(2);
void collisionAvoidance();

// New controller functions and variables
void calcOrder();
Eigen::Matrix3f buildCamGoal(Eigen::Vector3f az);
Eigen::Matrix2f matrix2d(int index1, int index2, Eigen::Matrix3f mat);

Eigen::Vector3f cam_goal, hand_goal, tgt;
Eigen::Matrix3f Rcam_goal, Rhand_goal;
Eigen::Vector2f w_hat;
Eigen::Matrix2f Rg_hat;

#endif


