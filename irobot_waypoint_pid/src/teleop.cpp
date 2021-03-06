// iRobot Create Teleoperation via Keyboard, using irobot_create_2_1 driver
// By Yan Zhang, 08/19/2015

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class RobotDriver
{
private:
  // Define the node handling
  ros::NodeHandle nh_;
  // Define the publisher
  ros::Publisher cmd_vel_pub_;

public:
  // ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    std::string robotname, topicname;
    nh.getParam("/robot/name", robotname);
    topicname = robotname+"/cmd_vel";
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(topicname.c_str(), 100);
  }

  bool driveKeyboard()
  {
    std::cout << "'s' to stop \n 'i' to move forward \n 'u' to move left forward \n 'o' to move right forward \n 'j' to move left backward \n 'k' to move backward \n 'l' to move right backward \n ',' to turn counter-clockwise \n '.' to turn clockwise \n" ;

    geometry_msgs::Twist base_cmd;
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.linear.z = 0;

    char cmd[50];
    while(ros::ok())
      {
        std::cin.getline(cmd, 50);
        if(cmd[0]!='s' && cmd[0]!='i' && cmd[0]!='u' && cmd[0]!='o' && cmd[0]!='j' && cmd[0]!='k' && cmd[0]!='l' && cmd[0]!=',' && cmd[0]!='.')
        {
	  std::cout << "unknown command:" << cmd << "\n";
	  continue;
        }
	
	// Stop
	if(cmd[0] == 's')
	{
	  base_cmd.linear.x = 0;   // positive x means moving forward
	  base_cmd.angular.z = 0;  // positive z means turing counter-clockwise
	}
	// Move Forward
	else if(cmd[0] == 'i')
	{
	  base_cmd.linear.x = 0.25;
	  base_cmd.angular.z = 0;
	}
	// Move Left Forward
	else if(cmd[0] == 'u')
	{
	  base_cmd.linear.x = 0.25;
	  base_cmd.angular.z = 0.5;
	}
	// Move Right Forward
	else if(cmd[0] == 'o')
	{
	  base_cmd.linear.x = 0.25;
	  base_cmd.angular.z = -0.5;
	}
	// Move Left Backward
	else if(cmd[0] == 'j')
	{
	  base_cmd.linear.x = -0.25;
	  base_cmd.angular.z = -0.5;
	}
	// Move Backward
	else if(cmd[0] == 'k')
	{
	  base_cmd.linear.x = -0.25;
	  base_cmd.angular.z = 0;
	}
	// Move Right Backward
	else if(cmd[0] == 'l')
	{
	  base_cmd.linear.x = -0.25;
	  base_cmd.angular.z = 0.5;
	}

	else if(cmd[0] == ',')
	{
	  base_cmd.linear.x = 0;
	  base_cmd.angular.z = 0.5;
	}

	else if(cmd[0] == '.')
	{
	  base_cmd.linear.x = 0;
	  base_cmd.angular.z = -0.5;
	}
	
	//Publish the command
	cmd_vel_pub_.publish(base_cmd);
      }
    return true;	
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  RobotDriver driver(nh);
  driver.driveKeyboard();

  return(0);
}
