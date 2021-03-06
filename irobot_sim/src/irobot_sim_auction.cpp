#include "sim_lib_auction.cpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Brain5_sim_auction");
  ros::NodeHandle n;

  std::string robotname, temp, result;
  n.getParam("/robot/name", robotname);

  rama_commander commander;
//  rama_robot robot;
  commander.loadAuctionTargets();
  commander.loadAuctionEntryList();
  commander.loadAuctionWaypointList();

  commander.showEntryList();
  commander.showWaypointList(3, 11);

  printf("Robot: Hi, I am %s ...\n", robotname.c_str());

  result = robotname + "/auction_choice";
  auction_choice_pub = n.advertise<std_msgs::Float32MultiArray>(result.c_str(), 1);
  result = robotname + "/current_choice";
  current_choice_pub = n.advertise<std_msgs::Float32MultiArray>(result.c_str(), 1);
  result = robotname + "/auction_req";
  auction_req_pub = n.advertise<std_msgs::Float32MultiArray>(result.c_str(), 1);
  result = robotname + "/auction_response";
  auction_response_pub = n.advertise<std_msgs::Float32MultiArray>(result.c_str(), 1);
  peer_auction_choice_sub = n.subscribe("Brain4/auction_choice", 1, &rama_commander::peerAuctionChoiceCallback, &commander);
  peer_current_choice_sub = n.subscribe("Brain4/current_choice", 1, &rama_commander::peerCurrentChoiceCallback, &commander);
  peer_auction_req_sub = n.subscribe("Brain4/auction_req", 1, &rama_commander::peerAuctionReqCallback, &commander);
  peer_response_sub = n.subscribe("Brain4/auction_response", 1, &rama_commander::peerAuctionResponseCallback, &commander);

  printf("Robot : Please press 'any key + enter' to start the waypoint tracking...\n");
  waitForEnter();

  int robot_wpt_count = 0;

  // Clear visit_history
  my_visit_history.setZero();
  peer_visit_history.setZero();

  int iii;
  for (iii = 0; iii < NUM_TARGET; iii++)
  {
    available_target_list[iii] = true;
  }

  current_target_done = true;
  robot_working_state = 1; // Set robot to "transit" state;

  while(ros::ok() && (robot_working_state != 3))
  {
    if (current_target_done)
    {
      commander.generateAuctionPosition();
      commander.proposeAuction();
    }

    if (robot_working_state == 3) {break;}

    commander.sendAuctionWaypoint();

//  In simulation, show which waypoint is heading to, wait for some time, show robot is there.
//  In simulation, according to robot_working_state, change ocam_world or last_wayponit_in_current_list for finding nearest entry when receiving acution request from peer robot.
    commander.generateWorkingPosition();

//  Pretend robot is going to waypoints
//  Robot has got to the waypoint;

    if (!flag_beat_in_transit)
    {
      if (robot_working_state == 1)
      { 
          commander.NoticePeerAuctionChoice();
      }

//  take image here


//  Record Position here
      commander.recordSimPosition();

      commander.nextAuctionWaypoint();
//    robot_wpt_count++;
    }
  }

  printf("Commander : Job finished!\n");
  return 0;
}

