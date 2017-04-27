#ifndef __SIM_LIB_AUCTION_cpp____
#define __SIM_LIB_AUCTION_cpp____
#include "sim_lib_auction.h"

using namespace std;

rama_commander::rama_commander(){
  wpt_count = 0;
  goal_target = 0;
  fx = 8;
  fy = 8;
  which_waypoint_in_current_list = 0;
  sim_count = 0;
  sim_time = 1.0;
  start_time = 0.0;
  duration = 0.0;
  ping_count = 0;
  fresh_start = true;
  auction_response_counter = 0;
}

void rama_commander::loadAuctionTargets()
{
  int line, col;
  ifstream file;
  file.open("/home/dukerama/hydro_catkin/src/irobot_sim/include/Auction/InitTargets.txt");
    
  if (!file.is_open()){
    cout << "No InitTargets.txt file in the /include/Auction folder!\n";
    return; 
  }

  for(line = 0; line < NUM_TARGET; line++){
    for (col = 0; col < 3; col++){
      file >> Targets(line, col);
    }
  }
  
  file.close();
}

void rama_commander::loadAuctionEntryList()
{
  int line, col, number_of_lines;

  int index_target;
  for (index_target = 1; index_target < NUM_TARGET+1; index_target++)
  {
    ifstream file;
    char name[255];
    std::stringstream ss;
    std::string filename;
    sprintf(name, "/home/dukerama/hydro_catkin/src/irobot_sim/include/Auction/EntryList%d.txt", index_target);
    ss << name;
    ss >> filename;
//    std::cout << filename << std::endl;
    number_of_lines = getNumberofLine(filename);
//    std::cout << number_of_lines << std::endl;

    file.open(filename.c_str());   
    if (!file.is_open()){
      printf("No such a file %s !\n", filename.c_str());
      return; 
    }

    Eigen::MatrixXf temp(number_of_lines, 3);
    for(line = 0; line < number_of_lines; line++){
      for (col = 0; col < 3; col++){   
        file >> temp(line, col);
      }
    } 
    file.close();
    EntryList[index_target] = temp;
  }
}

void rama_commander::loadAuctionWaypointList()
{
  int line, col, number_of_entries, number_of_waypoints;

  int index_target;
  for (index_target = 1; index_target < NUM_TARGET+1; index_target++)
  {
    ifstream file;
    char name[255];
    std::stringstream ss;
    std::string filename;
    sprintf(name, "/home/dukerama/hydro_catkin/src/irobot_sim/include/Auction/EntryList%d.txt", index_target);
    ss << name;
    ss >> filename;
//    std::cout << filename << std::endl;
    number_of_entries = getNumberofLine(filename);
//    std::cout << number_of_lines << std::endl;
    
    int index_entry;
    for (index_entry = 1; index_entry < number_of_entries+1; index_entry++)
    {
      ifstream file;
      char name[255];
      std::stringstream ss;
      std::string filename;
      sprintf(name, "/home/dukerama/hydro_catkin/src/irobot_sim/include/Auction/Waypoint0%d0%d.txt", index_target, index_entry);
      ss << name;
      ss >> filename;
      file.open(filename.c_str());   
      if (!file.is_open()){
        printf("No such a file %s !\n", filename.c_str());
        return; 
      }
//      std::cout << filename << std::endl;
      number_of_waypoints = getNumberofLine(filename);
      Eigen::MatrixXf temp(number_of_waypoints, 6); // Real file has 3 columns, although the third column is useless!!!
      for(line = 0; line < number_of_waypoints; line++){
        for (col = 0; col < 6; col++){   
          file >> temp(line, col);
        }
      } 
      file.close();
      WaypointList[index_target][index_entry] = temp;
    }
  }  
}

void rama_commander::showEntryList()
{
  int index_target;
  for (index_target = 1; index_target < NUM_TARGET+1; index_target++)
  {
    Eigen::MatrixXf mapp = EntryList[index_target];
    int row;
    std::cout << "EntryList[" << index_target << "] = " << "\n";
    for (row = 0; row < mapp.rows(); row++)
    { std::cout << EntryList[index_target](row, 0) << " " << EntryList[index_target](row, 1) << "\n"; }
  }
}

void rama_commander::showWaypointList(int index_target, int index_entry)
{
  Eigen::MatrixXf mapp = WaypointList[index_target][index_entry];
  int row;
  std::cout << "WaypointList[Target = " << index_target << "][Entry = " << index_entry << "] = " << "\n";
  for (row = 0; row < mapp.rows(); row++)
  { std::cout << WaypointList[index_target][index_entry](row, 0) << " " << WaypointList[index_target][index_entry](row, 1) << " "
              << WaypointList[index_target][index_entry](row, 2) << " " << WaypointList[index_target][index_entry](row, 3) << " "
              << WaypointList[index_target][index_entry](row, 4) << " " << WaypointList[index_target][index_entry](row, 5) << "\n"; }
}

void rama_commander::proposeAuction()
{
    if (!checkAllTargetDone())
    {
        my_chosen_entry = getAuctionNearestEntry(); // Choose nearest entry according to robot working state and available target list
        my_visit_history[my_chosen_entry.which_target-1] = my_chosen_entry.bid;
        my_visit_history[NUM_TARGET] = (float)(my_chosen_entry.which_target);
        std::cout << "Commander: My updated visit history: ";
        int ii;
	for (ii = 0; ii < NUM_TARGET+1; ii++)
        {
	    std::cout << my_visit_history[ii] << " ";
	    if (ii == NUM_TARGET)
            {
		std::cout << std::endl;
            }
        }	
        std::cout << "Commander: I want to propose an auction to peers!\n";
        robot_record.data.clear();
        for (ii = 0; ii < NUM_TARGET+1; ii++)
        { robot_record.data.push_back(my_visit_history[ii]); } 
//      Wait For Peer Response
        receive_peer_response = false;
        while (!receive_peer_response)
        {
          ros::spinOnce();
          auction_req_pub.publish(robot_record);
          ros::Duration(0.5).sleep();
        }
        doAuction_v2();
        fresh_start = false;
    }
    else
    {
        std::cout << "All targets are done! Job finished!" << std::endl;
        robot_working_state = 3;
    }
}

void rama_commander::doAuction_v2()
{
    int my_choice = (int)(my_visit_history[NUM_TARGET]+0.5);
    int peer_choice = (int)(peer_visit_history[NUM_TARGET]+0.5);
    std::cout << "Commander: My choice is " << my_choice << " and my peer's choice is " << peer_choice << std::endl;

    // Update available target list
//    available_target_list[my_choice-1] = false;
//    available_target_list[peer_choice-1] = false;

    Eigen::MatrixXf my_auction_waypointlist;

    if (my_choice == peer_choice) // We need auction!
    {
      std::cout << "Commander: Choice conflicts! Now I'm doing auction!\n";
      if (my_visit_history[my_choice-1] > peer_visit_history[peer_choice-1]) // I win the auction!
      {
        // Update my_auction_waypointlist
        std::cout << "Commander: I win the auction! I will choose the waypointlist target = " << my_chosen_entry.which_target
		  << " entry = " << my_chosen_entry.which_entry << "\n";
        my_auction_waypointlist = WaypointList[my_chosen_entry.which_target][my_chosen_entry.which_entry];
      }
      else								    // I lose the auction!
      {
        std::cout << "Commander: I lose the auction! I will find my backup choice." << "\n";
      // Find back-up choice
      	available_target_list[my_choice-1] = false; 
        if (!checkAllTargetDone())
        {
      // Re-compute the nearest target and entry
      	  my_chosen_entry = getAuctionNearestEntry();
          my_auction_waypointlist = WaypointList[my_chosen_entry.which_target][my_chosen_entry.which_entry];
          available_target_list[my_choice-1] = true;
        }
        else
        {
	  available_target_list[my_choice-1] = true; // Although I lost the auction, peer robot may not have visited this target! Leave this available for next auction.
          std::cout << "Commander: No available targets for me, all work done!" << "\n";
          no_work_for_me = true;
          robot_working_state = 3;
          while (available_target_list[my_choice-1])
          { ros::spinOnce(); ros::Duration(0.1).sleep(); }
        }
      }
    }
    else
    {
      std::cout << "Commander: No conflicts!" << "\n";
      my_auction_waypointlist = WaypointList[my_chosen_entry.which_target][my_chosen_entry.which_entry];
    }

    if (robot_working_state != 3)
    {
    	std::cout << "Commander: Deal! I'll go to target " << my_chosen_entry.which_target << " !\n"; 
    	current_waypointlist = my_auction_waypointlist;
    	current_target_done = false;
     }
}

void rama_commander::NoticePeerAuctionChoice()
{
        float my_choice = current_waypointlist(0,3);

    	// Notice your peer your choice! 
    	auction_choice_msg.data.clear();
    	int ii;
    	for (ii = 0; ii < 1; ii++)
    	{ auction_choice_msg.data.push_back(my_choice); } 
//  	Wait For Peer Response
    	receive_peer_choice = false;
    	while (!receive_peer_choice)
    	{
        	ros::spinOnce();
        	auction_choice_pub.publish(auction_choice_msg);
       		ros::Duration(0.5).sleep();
    	} 
    	available_target_list[my_chosen_entry.which_target-1] = false;  
}

void rama_commander::peerAuctionReqCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
    int ii = 0;
    for (std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
      peer_visit_history[ii] = *it;
      ii++;
    }  

    std::cout << "Commander: I heard my peer's auction request, his auction info is: ";

    for (ii = 0; ii < NUM_TARGET+1; ii++)
    {
	std::cout << peer_visit_history[ii] << " ";
	if (ii == NUM_TARGET)
        {
	    std::cout << std::endl;
        }
    } 

    my_chosen_entry = getAuctionNearestEntry(); // Choose nearest entry according to robot working state and available target list
    my_visit_history[my_chosen_entry.which_target-1] = my_chosen_entry.bid;
    my_visit_history[NUM_TARGET] = (float)(my_chosen_entry.which_target);
    std::cout << "Commander: My chosen nearest target info is: ";

    for (ii = 0; ii < NUM_TARGET+1; ii++)
    {
	std::cout << my_visit_history[ii] << " ";
	if (ii == NUM_TARGET)
        {
	    std::cout << std::endl;
        }
    }
    robot_record.data.clear();
    for (ii = 0; ii < NUM_TARGET+1; ii++)
    { robot_record.data.push_back(my_visit_history[ii]); }
    auction_response_pub.publish(robot_record);
}

void rama_commander::peerAuctionResponseCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
    if (fresh_start)
    {}
    else
    { last_peer_history = peer_visit_history; }

    int ii = 0;

    for (std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
      peer_visit_history[ii] = *it;
      ii++;
    }

    if (peer_visit_history == last_peer_history)
    {   
        std::cout << "Commander: Peer_visit_history is not updated compared to last record!" << std::endl; 
        auction_response_counter++;
	if (auction_response_counter > 5)
        { receive_peer_response = true; auction_response_counter = 0; }
    }
    else
    {
        std::cout << "Commander: I propose an auction and I heard my peer's response, his auction info is: ";

    	for (ii = 0; ii < NUM_TARGET+1; ii++)
    	{
		std::cout << peer_visit_history[ii] << " ";
		if (ii == NUM_TARGET)
        	{
		    std::cout << std::endl;
        	}
    	}	 
    	receive_peer_response = true;
        auction_response_counter = 0;
    }
}

void rama_commander::peerAuctionChoiceCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
  int ii = 0;
  for (std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
  {
      	peer_auction_choice[ii] = *it;
      	ii++;
  }     
  int peer_choice = (int)(peer_auction_choice(0)+0.5);

//      last_peer_history = peer_visit_history;
  std::cout << "Commander: I heard my peer's auction choice! He is going to target " << peer_choice << " !" << std::endl;

  available_target_list[peer_choice-1] = false; // Update available target list;

  current_choice_msg.data.clear();
  for (ii = 0; ii < 1; ii++)
  { current_choice_msg.data.push_back((float)(my_chosen_entry.which_target)); }
  current_choice_pub.publish(current_choice_msg);

  int my_choice = (int)(current_waypointlist(0,3)+0.5);

  if (peer_choice == my_choice && robot_working_state == 1)
  {
	std::cout << "Commander: I was beat during transit to target " << peer_choice << "!" << std::endl;
        if (!checkAllTargetDone())
        {
      // Re-compute the nearest target and entry
      	  my_chosen_entry = getAuctionNearestEntry();
          std::cout << "Commander: I rechoose my target: " << my_chosen_entry.which_target << " and entry: " << my_chosen_entry.which_entry << std::endl;
          current_waypointlist = WaypointList[my_chosen_entry.which_target][my_chosen_entry.which_entry];
          which_waypoint_in_current_list = 0;
          flag_beat_in_transit = true;
        }
        else
        {
          std::cout << "Commander: No available targets for me, all work done!" << "\n";
          no_work_for_me = true;
          robot_working_state = 3;
        }        
  }
  else if (peer_choice == my_choice && robot_working_state != 1)
  { std::cout << "Commander: Impossible! Check your codes!" << std::endl; }
}

void rama_commander::peerCurrentChoiceCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
  int ii = 0;
  for (std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
  {
      	peer_auction_choice[ii] = *it;
      	ii++;
  }     
  int peer_choice = (int)(peer_auction_choice(0)+0.5);
  std::cout << "Commander: I told my peer my auction choice and he heard it! He is currently detecting target " << peer_choice << " !" << std::endl;
  receive_peer_choice = true; 
}

// ******** getAuctionNearestEntry ******** //
// Receive robot_working_state as input, whether I am in "transit"(1) mode, in "busy"(2) mode.
// If I am in "transit", I'll find nearest entry (and bid) based on my current position returned from opti-track.
// If I am in "busy", I'll find nearest entry (and bid) based on current waypointlist and available entrylist.
NearestEntry rama_commander::getAuctionNearestEntry()
{
  NearestEntry value;
  int index_target, ii;
  float min_dist = 100.0;
  Eigen::VectorXf dist2entry(2);
  Eigen::VectorXi output(2);
  Eigen::MatrixXf temp;

  if (robot_working_state == 1 || robot_working_state == 3)
  {
//    flag_getCurrentPosition = 0;
//    while (flag_getCurrentPosition == 0)
//    {  ros::spinOnce(); ros::Duration(0.01).sleep(); }
  }
  else if (robot_working_state == 2)
  {
//    std::cout << "Hello!" << std::endl;
    last_waypoint_in_current_list(0) = current_waypointlist(current_waypointlist.rows()-1, 0);
    last_waypoint_in_current_list(1) = current_waypointlist(current_waypointlist.rows()-1, 1);
//    std::cout << "Hi!" << std::endl;
  }
//  else { std::cout << "Commander : Warning! Unrecognizable robot working state " << robot_working_state << " !\n"; }

  for (index_target = 1; index_target < NUM_TARGET+1; index_target++)
  {
    if (available_target_list[index_target-1])
    {
      temp = EntryList[index_target];
      for (ii = 0; ii < temp.rows(); ii++)
      {

        if (robot_working_state == 1) // If robot is in transit mode;
        {
          dist2entry(0) = temp(ii, 0) - ocam_world(0);
          dist2entry(1) = temp(ii, 1) - ocam_world(1);
          if (dist2entry.norm() < min_dist)
          {
            min_dist = dist2entry.norm();
            output(0) = index_target;
            output(1) = ii+1;
          }
        }
        else if (robot_working_state == 2) // If robot is in busy mode, then command has chosen a current_waypointlist as current mission;
        {
          dist2entry(0) = temp(ii, 0) - last_waypoint_in_current_list(0);
          dist2entry(1) = temp(ii, 1) - last_waypoint_in_current_list(1);
          if ((dist2entry.norm() + current_waypointlist(which_waypoint_in_current_list, 5)) < min_dist)
          {
            min_dist = dist2entry.norm() + current_waypointlist(which_waypoint_in_current_list, 5);
            output(0) = index_target;
            output(1) = ii+1;
          }
        }
      }
    }
  } 

  std::cout << "Commander: Nearest entry is found, target = " << output(0) << ", entry = " << output(1) << ", distance = " << min_dist << std::endl;

  value.which_target = output(0);
  value.which_entry = output(1);
  value.distance = min_dist;
  value.bid = 1/(1+min_dist);

  return value; // output(0) = min_index_target, output(1) = min_index_entry;  
}

void rama_commander::sendAuctionWaypoint()
{
  if (flag_beat_in_transit)
  { flag_beat_in_transit = false; }

  if (which_waypoint_in_current_list < current_waypointlist.rows())
  {
    waypoint = current_waypointlist.row(which_waypoint_in_current_list);
    if (which_waypoint_in_current_list == 0)
    { robot_working_state = 1; }
    else
    { robot_working_state = 2; }
  }
  else
  { std::cout << "Commander: Error! Current waypointlist is used up! I'll stop the robot!" << "\n"; }
}

void rama_commander::nextAuctionWaypoint()
{
  which_waypoint_in_current_list++;
  if (which_waypoint_in_current_list == current_waypointlist.rows())
  {
    if (!checkAllTargetDone())
    {
      current_target_done = true;
      which_waypoint_in_current_list = 0;
      robot_working_state = 1;
    }
    else
    {
      robot_working_state = 3;
      std::cout << "Commander: Congratulations! All work is done!" << "\n";
    }
  }
}

bool rama_commander::checkAllTargetDone()
{
  bool value = true;
  int ii;
  for (ii = 0; ii < NUM_TARGET; ii++)
  {
    if (available_target_list[ii]) { value = false; }
  }
  return value;
}

//************************* RAMA_COMMANDER FOR SIMULATION ***************************//

void rama_commander::generateAuctionPosition()
{
    if (sim_count == 0) // The first first stage.
    {
      ocam_world(0) = 1.2; // Set the initial robot position;
      ocam_world(1) = 0.0;
      std::cout << "Simulator: My initial simulated position is " << ocam_world(0) << " " << ocam_world(1) << "\n"; 
      sim_count++;
    }
    else
    {
      ocam_world(0) = waypoint[0]; ocam_world(1) = waypoint[1];
      std::cout << "Simulator: An auction is proposed! My simulated position is " << ocam_world(0) << " " << ocam_world(1) << "\n";
    }
}

void rama_commander::generateWorkingPosition()
{
  if (robot_working_state == 1)
  {
    int my_choice = (int)(current_waypointlist(0, 3));
    std::cout << "Simulator: I am in transit state! I am heading to [" << waypoint[0] << ", " << waypoint(1) << "] for target "
              << my_choice << "...\n";
    // Sleep for a while;
    duration = 0;
    start_time = ros::Time::now().toSec();
    while (duration < sim_time)
    {
      ros::spinOnce();
      duration = ros::Time::now().toSec() - start_time;
    }
    ocam_world(0) = waypoint[0]; ocam_world(1) = waypoint[1];
  }
  else if (robot_working_state == 2)
  {
    int my_choice = (int)(current_waypointlist(0, 3));
    std::cout << "Simulator: I am in busy state! I am heading to [" << waypoint[0] << ", " << waypoint(1) << "] for target "
              << my_choice << "...\n";
    // Sleep for a while;
    duration = 0;
    start_time = ros::Time::now().toSec();
    while (duration < sim_time)
    {
      ros::spinOnce();
      duration = ros::Time::now().toSec() - start_time;
    }
  }
}

void rama_commander::recordSimPosition()
{
  for (int i=0;i<6;i++)
  {
    outfile_sim << waypoint(i) << " ";
  }

  outfile_sim << "\n";
}

int getNumberofLine(std::string filename)
{
  int number_of_lines = 0;
  ifstream file;
  std::string line;

  file.open(filename.c_str());
    
  if (file.is_open()){
    while(std::getline(file, line))
    {
      ++number_of_lines;
    }
    return number_of_lines;
  }
  else
  { return 0; }
}

void waitForEnter()
{
  char Enter;
  std::cin >> Enter;
}


#endif
