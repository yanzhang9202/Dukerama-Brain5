#ifndef __WPSLIB_cpp
#define __WPSLIB_cpp

#include "wpslib.h"

// Read matrix in \include\Waypoints.txt file. The matrix should be like this:
// 1.000 0.000 0.000
// 0.000 1.000 0.000
 
using namespace std;

void loadWaypoints()
{
  int line, col;
  ifstream file;
  file.open("/home/dukerama/hydro_catkin/src/irobot_waypoint_commander/include/Config/Waypoints.txt");
    
  if (!file.is_open()){
    cout << "No Waypoints.txt file in the /include/Config folder!\n";
    return; 
  }

  for(line = 0; line < NUM_WPS; line++){
    for (col = 0; col < 3; col++){
      file >> Waypoints(line, col);
    }
  }
  
  file.close();
}

void loadTargets()
{
  int line, col;
  ifstream file;
  file.open("/home/dukerama/hydro_catkin/src/irobot_waypoint_commander/include/Config/InitTargets.txt");
    
  if (!file.is_open()){
    cout << "No InitTargets.txt file in the /include/Config folder!\n";
    return; 
  }

  for(line = 0; line < NUM_TARGET; line++){
    for (col = 0; col < 3; col++){
      file >> InitTargets(line, col);
    }
  }
  
  file.close();
}

void WpsRqstCallback(const geometry_msgs::Point msg){
  wps_request = msg.x;
  wps_done = msg.y;

  if (wps_request == 1 && wps_done == 0 && wps_count < NUM_WPS){
    printf("Commander : Request received! Please go to the %dth waypoint [%f, %f, %f]...\n", wps_count+1, Waypoints(wps_count, 0), Waypoints(wps_count, 1), Waypoints(wps_count, 2));
    wps_msg.x = Waypoints(wps_count, 0);
    wps_msg.y = Waypoints(wps_count, 1);
    wps_msg.z = Waypoints(wps_count, 2);
    wps_pub.publish(wps_msg);

    tgt_msg.x = InitTargets(goal_target, 0);
    tgt_msg.y = InitTargets(goal_target, 1);
    tgt_msg.z = InitTargets(goal_target, 2);
    target_pub.publish(tgt_msg);
  }

  if (wps_request == 0 && wps_done == 0 && wps_count < NUM_WPS){
    printf("Commander : Good! You heard me! Go there and detect the %dth target. I guess it's around [%f, %f, %f]...\n", goal_target+1, InitTargets(goal_target, 0), InitTargets(goal_target, 1), InitTargets(goal_target, 2));
  } 
 
  if (wps_request == 0 && wps_done == 1 && wps_count < NUM_WPS){
    printf("Commander : Nice job!\n");
    wps_count++;
  }

  if (wps_request == 1 && wps_count >= NUM_WPS){
    printf("Commander : All waypoints have been sent!\n");
    all_sent = 1;
    all_sent_msg.x = all_sent;
    all_sent_pub.publish(all_sent_msg);
  }
}

#endif 

