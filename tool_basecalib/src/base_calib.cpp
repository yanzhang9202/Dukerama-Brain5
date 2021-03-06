#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#include "maestro_serial_driver.cpp"

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

void sendServoSpeed();
void initialServoTarget();
void vrpnCallback(const geometry_msgs::TransformStamped msg);
void recordCamPose_forward();
void recordCamPose_center();
void recordCamPose_rotate();
void robotMoveForward();
void robotRotate();
void stopBase();
void waitForEnter();
void sendServoTarget(float target);

Eigen::VectorXf q_h(4), ohand_world(3);
geometry_msgs::Twist  u_msg;

ros::Subscriber robot_vrpn_sub;
ros::Publisher  base_pub;

std::ofstream outfile_forward("/home/dukerama/hydro_catkin/src/tool_basecalib/include/positions/BaseCalib_forward.txt");
std::ofstream outfile_center("/home/dukerama/hydro_catkin/src/tool_basecalib/include/positions/BaseCalib_center.txt");
std::ofstream outfile_rotate("/home/dukerama/hydro_catkin/src/tool_basecalib/include/positions/BaseCalib_rotate.txt");

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Base_Calib");
  ros::NodeHandle n;

  robot_vrpn_sub = n.subscribe("Brain5Camera/pose", 1, vrpnCallback);
  base_pub = n.advertise<geometry_msgs::Twist>("Brain5/cmd_vel", 1);  

  ros::Rate loop_rate(1);

  sendServoSpeed();
  initialServoTarget();

  printf("Robot : Please press 'any key + enter' to start the base calibration...\n");
  waitForEnter();

  int forward_count = 0;
  int center_count = 0;
  int rotate_count = 0;

  std::cout << "Robot: Calibrating Rot_r2n..." << std::endl;
  while (forward_count < 4)
  {
      robotMoveForward();
      recordCamPose_forward();
      forward_count ++;
      std::cout << "Position " << forward_count << std::endl;
  }

  std::cout << "Robot: Calibrating servo and base center in hand frame..." << std::endl;
  recordCamPose_center();
  int resolution = 40;
  float delta = 900/resolution;
  sendServoTarget(1050.0);
  while (center_count < 40)
  {
      sendServoTarget(1050.0 + ((float)center_count)*delta);
      recordCamPose_center();
      center_count ++;
      std::cout << "Position " << center_count << std::endl;
  }

  initialServoTarget();
  std::cout << "Robot: Calibrating servo position in base..." << std::endl;
  while (rotate_count < 50){
	recordCamPose_rotate();
	robotRotate();
	rotate_count++;
	std::cout << "Position: " << rotate_count << std::endl;
  }

  printf("Robot : Job finished!\n");
  outfile_forward.close();
  outfile_center.close();
  outfile_rotate.close();
  return 0;
}

void sendServoSpeed()
{
   int fd = maestroConnect();
   maestroSetSpeed(fd, 0, 5);
   close(fd); 
}

void initialServoTarget()
{
   int fd = maestroConnect();
   maestroSetTarget(fd, 0, 6000);
   while (GetMovingState(fd,0)) {}
   close(fd); 
}

void sendServoTarget(float target)
{
   int fd = maestroConnect();
   maestroSetTarget(fd, 0, target*4);
   while (GetMovingState(fd,0)) {}
   close(fd); 
}

void vrpnCallback(const geometry_msgs::TransformStamped msg){
// Should add a condition that tgt_world data is valid;
  if (1)
//  if (wpt_request==0 && wpt_done==0)
  {
    q_h[0] =msg.transform.rotation.x;
    q_h[1] =msg.transform.rotation.y;
    q_h[2] =msg.transform.rotation.z;
    q_h[3] =msg.transform.rotation.w;

    ohand_world[0] = msg.transform.translation.x;
    ohand_world[1] = msg.transform.translation.y;
    ohand_world[2] = msg.transform.translation.z;
  }
}

void recordCamPose_forward()
{
  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce();

  for (int i=0;i<3;i++)
  {
    outfile_forward << ohand_world(i) << " ";
  }
  for (int i=0;i<4;i++)
  {
    outfile_forward << q_h(i) << " ";
  }
  outfile_forward << "\n";
}


void recordCamPose_center()
{
  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce();

  for (int i=0;i<3;i++)
  {
    outfile_center << ohand_world(i) << " ";
  }
  for (int i=0;i<4;i++)
  {
    outfile_center << q_h(i) << " ";
  }
  outfile_center << "\n";
}

void recordCamPose_rotate()
{
  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce();

  for (int i=0;i<3;i++)
  {
    outfile_rotate << ohand_world(i) << " ";
  }
  for (int i=0;i<4;i++)
  {
    outfile_rotate << q_h(i) << " ";
  }
  outfile_rotate << "\n";
}

void robotMoveForward()
{
  u_msg.linear.x = 0.1;
  u_msg.angular.z = 0;

  base_pub.publish(u_msg);
  ros::Duration(2.0).sleep();
  stopBase();
}

void robotRotate()
{
  u_msg.linear.x = 0;
  u_msg.angular.z = 0.2;

  base_pub.publish(u_msg);
  ros::Duration(2.0).sleep();
  stopBase();
}

void stopBase()
{
      u_msg.angular.z = 0;
      u_msg.linear.x = 0;
      base_pub.publish(u_msg);
}

void waitForEnter()
{
    char enter;
    std::cin >> enter;
}

