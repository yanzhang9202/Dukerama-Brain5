#include <ros/ros.h>
#include <iostream>
#include <maestro_servo_driver/MaestroServerAction.h>
#include <actionlib/client/simple_action_client.h>
#include "maestro_serial_driver.cpp"

typedef actionlib::SimpleActionClient<maestro_servo_driver::MaestroServerAction> Client;

int main(int argc, char**argv)
{
  ros::init(argc,argv,"maestro_servo_client");
  ros::NodeHandle n;

  Client client("maestro_servo_server", true);
  client.waitForServer();
  maestro_servo_driver::MaestroServerGoal goal;
  
//  double pulse_width = 1500;
//  goal.servo_target = pulse_width * 4;

  int fd = maestroConnect();
  maestroSetSpeed(fd, 0, 5);
  maestroSetTarget(fd, 0, 6000);
  close(fd);

  while(ros::ok())
  {
    std::cout << "Input your target position from 1000 to 2000(us):" << std::endl;
    std::cin >> goal.servo_target;
    goal.servo_target = goal.servo_target * 4;
    client.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = client.waitForResult(ros::Duration(10.0)); 
    if (finished_before_timeout)
    {
      printf("Target position %f finished!\n", goal.servo_target/4);
    }
    else
    {ROS_INFO("Action did not finish before the time out.");}
  }

  return 0;
}
