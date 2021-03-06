#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include <geometry_msgs/TransformStamped.h>

#include <maestro_servo_driver/MaestroServerAction.h>
#include <actionlib/client/simple_action_client.h>
#include "maestro_serial_driver.cpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

typedef actionlib::SimpleActionClient<maestro_servo_driver::MaestroServerAction> Client;
#define PI 3.1415926

void waitForEnter(){
	char Enter;
	std::cin >> Enter;
}

Eigen::VectorXf pos(3), quat(4);

void vrpnCallback(const geometry_msgs::TransformStamped msg)
{
  quat[0] = msg.transform.rotation.x;
  quat[1] = msg.transform.rotation.y;
  quat[2] = msg.transform.rotation.z;
  quat[3] = msg.transform.rotation.w;

  pos[0] = msg.transform.translation.x;
  pos[1] = msg.transform.translation.y;
  pos[2] = msg.transform.translation.z;
}

int main(int argc, char**argv)
{
  ros::init(argc,argv,"servo_calib_client");
  ros::NodeHandle n;

  // Print out the data in "pulse_width(us), rotated angle(degree)" format in the \include\calib.txt file
  std::ofstream outfile("/home/dukerama/hydro_catkin/src/maestro_servo_driver/include/calib.txt");
  
  ros::Subscriber sub_vrpn = n.subscribe("/Brain5Camera/pose", 1, vrpnCallback);
  
  Client client("maestro_servo_server", true);
  client.waitForServer();
  maestro_servo_driver::MaestroServerGoal goal;

  // Drive the servo to home position, and record home position.
  printf("Servo goes to home position.\n");
  int fd = maestroConnect();
  maestroGoHome(fd, 0);
  ros::Duration(3.0).sleep();

  int position = maestroGetPosition(fd, 0);
  close(fd);

  ros::spinOnce();
  std::cout << quat << std::endl;
  Eigen::Quaternionf home(quat[3], quat[0], quat[1], quat[2]);

  printf("Current position is %d.\n", position/4);
  outfile << position/4 << " " << home.angularDistance(home)/PI*180 << "\n";

  printf("Please press any keys plus Enter to continue...\n");
  waitForEnter();

  int num_divide = 50;
  int delta_pulsewidth = 1000/num_divide;
  int count = 0;

  while(ros::ok() && count<=num_divide)
  {
    goal.servo_target = (count * delta_pulsewidth + 1000) * 4;

    client.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = client.waitForResult(ros::Duration(5.0)); 
    if (finished_before_timeout)
    {
      printf("Target position %f achieved!\n", goal.servo_target/4);
    }
    else
    {ROS_INFO("Action did not finish before the time out.");}

    ros::Duration(1.5).sleep();

    int fd = maestroConnect();
    int position = maestroGetPosition(fd, 0);
    close(fd);
    printf("Current position is %d.\n", position/4);

    ros::spinOnce();
    Eigen::Quaternionf quat_new(quat[3], quat[0], quat[1], quat[2]);
    float angulardistance = quat_new.angularDistance(home)/PI*180;
    printf("The angular distance relative to home position is %f.\n", angulardistance);
    outfile << position/4 << " " << angulardistance << "\n";

    count++;
  }

  outfile.close();
  return 0;
}



