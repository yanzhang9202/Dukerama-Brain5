#include "pid_controller.cpp"

#include "maestro_serial_driver.cpp"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

#include <stdio.h>
#include <math.h>

#include <geometry_msgs/Twist.h>

#define PI 3.14159265

#define Camera_Orientation_Error_Tolerance 1.0
#define Base_Orientation_Error_Tolerance 1.5
#define Waypoint_Error_Tolerance 0.025

int wpt_request = 1;
int wpt_done = 1;

int raw_input = 0;

float servo_position, servo_cos, servo_sin, error_angle;
double start_time = 0, sec = 7, duration = 0, waitforWpt = 10, waitforServo = 5, waitforBase = 10;
float flag_forward_backward = 1; // 1 - Robot moving forward to the waypoint; -1 - Robot moving backward to the waypoint.
float BaseError;
int counter = 0;
bool exceedLimit = false, flag_exceedLimit = false;

Eigen::VectorXf q_h(4), ocam_hand(3), ocam_world(3), ohand_world(3), tgt_world(3), tgt_cam(3), waypoint(3), wpt2tgt(3), ocam2wpt(3), ocam2wpt2D(2), waypointError(2);
Eigen::VectorXf wpt2tgt_robot(3), ocam2wpt_world(3), ocam2wpt2D_world(2);
Eigen::Matrix3f Rot_h2w, Rot_c2h, Rot_c2w, Rot_w2c, Rot_r2n, Rot_n2h, Rot_r2w, Rot_w2r;

Eigen::VectorXf servo_center_world(3), base_center_world(3), ohand2servo_hand(3), bctr2ocam(3), bctr2ocam2D(2), bctr2wpt(3), bctr2wpt2D(2);

// Request cooperate with robot base, publish on "/cmd_vel" topic
ros::Publisher base_pub;
ros::Publisher pid_state_pub;

geometry_msgs::Twist  u_msg;
geometry_msgs::Point  pid_state_msg;

void vrpnCallback(const geometry_msgs::TransformStamped msg);
void servoCallback(const geometry_msgs::Point msg);
void stateCallback(const geometry_msgs::Point msg);
void targetCallback(const geometry_msgs::Point msg);
void waypointCallback(const geometry_msgs::Point msg);

void adjustBase();
void goingWaypoint(float Wpt_Tolerance);
void ifExceedServoLimit();
void getServoError();
bool sendServoInput();
float getBaseError();
void ifClosetoWpt();
void stopBase();

void sendServoSpeed();
void initialServoTarget();
void BaseBackUp();

void getServoPosition();

bool checkBaseBoundary();
void ifWptclosetoBase();

Eigen::VectorXf getWaypointError();

//pid_controller base_pid(0.5, 0, 0, 0.4, -0.4 , Base_Orientation_Error_Tolerance/180*PI); // input : [Kp, Ki, Kd, ul, ll, tolerance(in rad)];
pid_controller base_pid(1.0, 0, 0, 0.5, -0.5 , Base_Orientation_Error_Tolerance/180*PI); // input : [Kp, Ki, Kd, ul, ll, tolerance(in rad)];
pid_controller wpt_dist_pid(0.3, 0, 0, 0.2, -0.2, Waypoint_Error_Tolerance);
pid_controller wpt_ori_pid(0.5, 0, 0, 0.4, -0.4, Base_Orientation_Error_Tolerance/180*PI);

bool flag_i_turned = false;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_servo_node");
  ros::NodeHandle n;
//  ros::Rate loop_rate(50);

  std::string robotname, publishname;
  n.getParam("/robot/name", robotname);
  publishname = robotname + "/state";
  ros::Subscriber robot_state_sub = n.subscribe(publishname.c_str(), 1, stateCallback);
  publishname = robotname + "/target";
  ros::Subscriber robot_target_sub = n.subscribe(publishname.c_str(), 1, targetCallback);
  publishname = robotname + "/waypoint";
  ros::Subscriber robot_waypoint_sub = n.subscribe(publishname.c_str(), 1, waypointCallback);
//  publishname = robotname + "/servo_position";
//  ros::Subscriber robot_servo_sub = n.subscribe(publishname.c_str(), 1, servoCallback);
  ros::Subscriber robot_vrpn_sub = n.subscribe("Brain5Camera/pose", 1, vrpnCallback);
  publishname = robotname + "/cmd_vel";
  base_pub = n.advertise<geometry_msgs::Twist>(publishname.c_str(), 1);
  publishname = robotname + "/pid_state";
  pid_state_pub = n.advertise<geometry_msgs::Point>(publishname.c_str(), 1);

  if (raw_input == 1) // Choose raw image calibration results
  {
      Rot_c2h << -0.2113,  0.0064, -0.9774,
                  0.9772, -0.0199, -0.2114,
                 -0.0208, -0.9998, -0.0020; 
 
      ocam_hand << -0.0655203, -0.0244237, 0.0046857;
      std::cout << "Using raw image calibration results..." << "\n" << std::endl;
  }
  else
  {
//      Rot_c2h << -0.2115,  0.0016, -0.9774,
//                  0.9771, -0.0249, -0.2115,
//                 -0.0247, -0.9997,  0.0037;

//      ocam_hand << -0.06156, -0.02345, 0.00723;

      Rot_c2h <<  0.0918,  0.0170,  0.9956,
                 -0.9921,  0.0869,  0.0900,
                 -0.0850, -0.9961,  0.0248; 
      ocam_hand << 0.0750, -0.0221, 0.0023;

      std::cout << "Using rectified image calibration results..." << "\n" << std::endl; 
  }

// Servo_Base calibration is done on Oct 29th. Redefine robots need recalibration.
//  Rot_r2n <<  0.7834,  0.6215, 0,
//	     -0.6215,  0.7834, 0,
//    	      0,       0,      1;
  Rot_r2n <<  0.6470,  0.7625, 0,
	     -0.7625,  0.6470, 0,
    	      0,       0,      1;

  ohand2servo_hand << -0.02236, -0.02441, 0;

  base_pid.clearhistory();
  wpt_dist_pid.clearhistory();
  wpt_ori_pid.clearhistory();

  sendServoSpeed();
  initialServoTarget();

  getServoPosition();
  std::cout << "Servo position: " << servo_position << std::endl;

  while( ros::ok() )
  {
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();

    if (wpt_request == 0 && wpt_done == 0)
    { 
      // The first time adjust Base to the waypoint
// ******************************* First time *********************************//   
      counter = 0;
      BaseError = getBaseError(); // unit: rad
      while(fabs(BaseError/PI*180) > Base_Orientation_Error_Tolerance && counter < 3)
      { 
//        std::cout << ocam_world(0) << " " << ocam_world(1) << " " << ocam_world(2) << std::endl;
//        std::cout << waypoint(0) << " " << waypoint(1) << " " << waypoint(2) << std::endl;
//        ifClosetoWpt();
	ifWptclosetoBase();
        if (flag_i_turned)
        { base_pid.setParam(1.0, 0.2, 0.1, 0.6, -0.6, PI/180); }
        adjustBase();
//        ifExceedServoLimit();
        if (flag_i_turned)
        {
          base_pid.setParam(1.0, 0.2, 0.1, 0.5, -0.5 , Base_Orientation_Error_Tolerance/180*PI);
          flag_i_turned = false;
        }
        adjustBase();
//        ifExceedServoLimit();
        getServoError();
        std::cout << "Servo Error: " << error_angle << std::endl;
        start_time = ros::Time::now().toSec();
        while (fabs(error_angle) > Camera_Orientation_Error_Tolerance)
        {
          duration = ros::Time::now().toSec() - start_time;
          if (duration > waitforServo)
          {
            sendServoSpeed(); 
            break;
          }
          flag_exceedLimit = sendServoInput();
          if (flag_exceedLimit)
          { 
            flag_forward_backward = -flag_forward_backward;
            std::cout << "Robot : I am turning around!\n" << std::endl;
            flag_i_turned = true;
            break;
          }
          ros::Duration(0.5).sleep();
        }      
//        std::cout << "Robot : The " << counter+1 << "th time, Subprocess 2 has been finished...\n";

        BaseError = getBaseError();
        counter++;
      }

        goingWaypoint(6*Waypoint_Error_Tolerance);

// *************************** Second Time ******************************* // 
      counter = 0;
      BaseError = getBaseError(); // unit: rad
      while(fabs(BaseError/PI*180) > Base_Orientation_Error_Tolerance && counter < 3)
      {
//        ifClosetoWpt();
	ifWptclosetoBase();
        if (flag_i_turned)
        { base_pid.setParam(1.0, 0.2, 0.1, 0.6, -0.6, PI/180); }
        adjustBase();
//        ifExceedServoLimit();
        if (flag_i_turned)
        {
          base_pid.setParam(1.0, 0.2, 0.1, 0.5, -0.5 , Base_Orientation_Error_Tolerance/180*PI);
          flag_i_turned = false;
        }
        adjustBase();
//        ifExceedServoLimit();

        getServoError();
        std::cout << "Servo Error: " <<error_angle << std::endl;
        start_time = ros::Time::now().toSec();
        while (fabs(error_angle) > Camera_Orientation_Error_Tolerance)
        {
          duration = ros::Time::now().toSec() - start_time;
          if (duration > waitforServo)
          {
            sendServoSpeed(); 
            break;
          }
          flag_exceedLimit = sendServoInput();
          if (flag_exceedLimit)
          { 
            flag_forward_backward = -flag_forward_backward;
            std::cout << "Robot : I am turning around!\n" << std::endl;
            flag_i_turned = true;
            break;
          }
          ros::Duration(0.5).sleep();
        }      
        BaseError = getBaseError();
        counter++;
      }

      goingWaypoint(Waypoint_Error_Tolerance);

      getServoError();
      std::cout << "Servo Error: " <<error_angle << std::endl;
      start_time = ros::Time::now().toSec();
      while (fabs(error_angle) > Camera_Orientation_Error_Tolerance)
      {
        duration = ros::Time::now().toSec() - start_time;
        if (duration > waitforServo)
        {
          sendServoSpeed(); 
          break;
        }
        flag_exceedLimit = sendServoInput();
        if (flag_exceedLimit)
        { 
          flag_forward_backward = -flag_forward_backward;
          std::cout << "Robot : I am turning around!\n" << std::endl;
          break;
        }
        ros::Duration(0.5).sleep();
      }  
      getServoError();
      std::cout << "Servo Error: " << error_angle << std::endl;

// ********************************* End ****************************// 
      ros::spinOnce();
      ros::spinOnce();
      std::cout << "ocam_world: " << ocam_world(0) << " " << ocam_world(1) << " " << ocam_world(2) << "\n"; 
      std::cout << "waypoint: " << waypoint(0) << " " << waypoint(1) << " " << waypoint(2) << "\n";
      
      wpt_done  = 1;
      pid_state_msg.x = 0;
      pid_state_msg.y = 1;
      pid_state_msg.z = 0;
      pid_state_pub.publish(pid_state_msg);
      pid_state_pub.publish(pid_state_msg);
      pid_state_pub.publish(pid_state_msg);
    }
  }

  return 0;
}

void stateCallback(const geometry_msgs::Point msg)
{
  wpt_request = msg.x;
  wpt_done = msg.y;
}

void targetCallback(const geometry_msgs::Point msg)
{
  tgt_world(0) = msg.x;
  tgt_world(1) = msg.y;
  tgt_world(2) = msg.z;
  std::cout << "I heard" << tgt_world(0) << " " << tgt_world(1) << " " << tgt_world(2) << "\n";
}

void waypointCallback(const geometry_msgs::Point msg)
{
  waypoint(0) = msg.x;
  waypoint(1) = msg.y;
  waypoint(2) = msg.z;
//  std::cout << "I heard" << waypoint << "\n";
}

//void servoCallback(const geometry_msgs::Point msg)
//{
//  servo_position = msg.x;
//}

void vrpnCallback(const geometry_msgs::TransformStamped msg){
// Should add a condition that tgt_world data is valid;
  if (1)
//  if (wpt_request==0 && wpt_done==0)
  {
    q_h[0] =msg.transform.rotation.x;
    q_h[1] =msg.transform.rotation.y;
    q_h[2] =msg.transform.rotation.z;
    q_h[3] =msg.transform.rotation.w;
  
    Eigen::Quaternionf quat_h(q_h[3], q_h[0], q_h[1], q_h[2]);

    Rot_h2w = quat_h.matrix();
    Rot_c2w = Rot_h2w * Rot_c2h;
    Rot_w2c = Rot_c2w.transpose();

    ohand_world[0] = msg.transform.translation.x;
    ohand_world[1] = msg.transform.translation.y;
    ohand_world[2] = msg.transform.translation.z;

    ocam_world = Rot_h2w * ocam_hand + ohand_world;
  }
}

float getBaseError()
{
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce(); 

    getServoPosition();

    servo_cos = cos((servo_position-1500)/500*PI); // To make sure this conversion is valid, servo_position should be in range [1100, 1900];
    servo_sin = sin((servo_position-1500)/500*PI);

    Rot_n2h << servo_cos, servo_sin, 0,
	      -servo_sin, servo_cos, 0,
    	       0,         0,         1;

    Rot_r2w = Rot_h2w * Rot_n2h * Rot_r2n;
//    std::cout << Rot_r2w << "\n" << std::endl;
    Rot_w2r = Rot_r2w.transpose();

    ocam2wpt = Rot_w2r * (waypoint - ocam_world) * flag_forward_backward;
    //return (atan2(ocam2tgt[1], ocam2tgt[0])/PI*180);
//    std::cout << ocam2wpt(0) << " " << ocam2wpt(1) << " " << ocam2wpt(2) << "\n";
//    std::cout << ocam2wpt(0) << " " << ocam2wpt(1) << " " << ocam2wpt(2) << "\n";
    return (atan2(ocam2wpt[1], ocam2wpt[0]));
}

void ifClosetoWpt()
{
  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce(); 
  ocam2wpt_world = waypoint - ocam_world;
  ocam2wpt2D_world(0) = ocam2wpt_world(0);
  ocam2wpt2D_world(1) = ocam2wpt_world(1);
  std::cout << "Distance : " << ocam2wpt2D_world.norm() << std::endl;
  if (ocam2wpt2D_world.norm() < 0.13 )
  {
    std::cout << "Too closed to waypoint!" << std::endl;
    u_msg.linear.x = flag_forward_backward * (-1) * 0.05;
    u_msg.angular.z = 0;
    base_pub.publish(u_msg);
    ros::Duration(1.0).sleep();
    stopBase();
  }
}

void adjustBase()
{
    BaseError = getBaseError();
    start_time = ros::Time::now().toSec();
    while(fabs(BaseError/PI*180) > Base_Orientation_Error_Tolerance)
    {
      duration = ros::Time::now().toSec() - start_time;
      if (duration > waitforBase)
      { 
        BaseBackUp();
        start_time = ros::Time::now().toSec();
        duration = 0;
      }
      base_pid.setError(BaseError);
      u_msg.angular.z = base_pid.computePID();
//      std::cout << u_msg.angular.z << "\n";
      u_msg.linear.x = 0;
      base_pub.publish(u_msg);
      BaseError = getBaseError();
      std::cout << "Base Error: " << BaseError/PI*180 << " In radian: " << BaseError << " flag: " << flag_forward_backward << "\n";
      ros::Duration(0.1).sleep();
    }

    stopBase();
    base_pid.clearhistory();
}

void ifExceedServoLimit()
{
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();

    getServoPosition();

    servo_cos = cos((servo_position-1500)/500*PI); // To make sure this conversion is valid, servo_position should be in range [1100, 1900];
    servo_sin = sin((servo_position-1500)/500*PI);

    Rot_n2h << servo_cos, servo_sin, 0,
	      -servo_sin, servo_cos, 0,
    	       0,         0,         1;

    Rot_r2w = Rot_h2w * Rot_n2h * Rot_r2n;
//    std::cout << Rot_r2w << "\n" << std::endl;
    Rot_w2r = Rot_r2w.transpose();

    wpt2tgt_robot = Rot_w2r * (tgt_world - waypoint);
    if (atan2(wpt2tgt_robot(1), wpt2tgt_robot(0)) < -1*PI/4 && atan2(wpt2tgt_robot(1), wpt2tgt_robot(0)) > -3*PI/4)
    { flag_forward_backward = -flag_forward_backward; 
      std::cout << "Exceed Servo Limit!\n"; } 

    adjustBase();

}

void goingWaypoint(float Wpt_Tolerance)
{
      std::cout << "Going to waypoint!\n";
      waypointError = getWaypointError();
      std::cout << waypointError(0) << " " << waypointError(1) << std::endl;
      start_time = ros::Time::now().toSec();
      while( fabs(waypointError(0)) > Wpt_Tolerance)
      {
        duration = ros::Time::now().toSec() - start_time;
        if (duration > waitforWpt)
        { break; }

        wpt_dist_pid.setError(waypointError(0));
        u_msg.linear.x = wpt_dist_pid.computePID();
//        wpt_ori_pid.setError(waypointError(1));
//        u_msg.angular.z = wpt_ori_pid.computePID();
        u_msg.angular.z = 0;
        base_pub.publish(u_msg);
        waypointError = getWaypointError();
        std::cout << waypointError(0) << " " << waypointError(1) << std::endl;
        ros::Duration(0.1).sleep();
      }

      stopBase();
      wpt_dist_pid.clearhistory();
      wpt_ori_pid.clearhistory();
}

void getServoError()
{
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    wpt2tgt = Rot_w2c * (tgt_world - waypoint);
//    std::cout << tgt_world << std::endl;
//    std::cout << waypoint << std::endl;
//    std::cout << Rot_c2w << std::endl;
    error_angle = atan2(-wpt2tgt[0], wpt2tgt[2])/PI*180;
}

bool sendServoInput()
{
    getServoError();

    getServoPosition();

    while(servo_position < 1000 || servo_position > 2000)
    {
	std::cout << "Invalid servo position: " << servo_position << "! Request again!" << std::endl;
	getServoPosition();
	ros::Duration(0.1).sleep();
    }

    exceedLimit = false;
    if (fabs(error_angle) > Camera_Orientation_Error_Tolerance)
    {  
      int servo_delta = round(error_angle/180*2000);

      std::cout << "error_angle:" << error_angle << ", servo_delta:" << servo_delta << ", servo position:" << servo_position <<  std::endl; 

      if (error_angle < 0)
      {
//        std::cout << error_angle << "\n";
        float servo_target = servo_position*4 + servo_delta;

        if ((servo_position + servo_delta/4)<1125)
        {
          error_angle = 360 + error_angle;
          servo_delta = round(error_angle/180*2000);
          if ((servo_position + servo_delta/4 < 1875))
          { servo_target = servo_position*4 + servo_delta; }
          else
          { 
            servo_target = 6000; 
            printf("Servo : Request an impossible position.\n");
            exceedLimit = true;
//            flag_forward_backward = -flag_forward_backward;
            
          } 
//          printf("Servo : Request an impossible position %f.\n", servo_position + servo_delta/4);
//          servo_target = 4400; 
//          flag_forward_backward = -flag_forward_backward;
//          servo_target = 6000;
        }

        int fd = maestroConnect();
        maestroSetTarget(fd, 0, servo_target);
        while (GetMovingState(fd,0)) {}
        close(fd);
      } 

      if (error_angle > 0)
      {
//        std::cout << error_angle << "\n";
        float servo_target = servo_position*4 + servo_delta;

        if ((servo_position + servo_delta/4)>1875)
        {
          error_angle = error_angle - 360;
          servo_delta = round(error_angle/180*2000);
          if ((servo_position + servo_delta/4 > 1125))
          { servo_target = servo_position*4 + servo_delta; }
          else
          { 
            servo_target = 6000; 
            printf("Servo : Request an impossible position.\n");
//            flag_forward_backward = -flag_forward_backward;
            exceedLimit = true;
          } 
//          printf("Servo : Request an impossible position %f.\n", servo_position + servo_delta/4);
//          servo_target = 7600;
//          flag_forward_backward = -flag_forward_backward;
//          servo_target = 6000;
        }

        int fd = maestroConnect();
        maestroSetTarget(fd, 0, servo_target);
        while (GetMovingState(fd,0)) {}
        close(fd); 
      }
    }
  return exceedLimit;
}

Eigen::VectorXf getWaypointError()
{
    Eigen::VectorXf wptError(2);
    ros::spinOnce(); 

    getServoPosition(); 

    servo_cos = cos((servo_position-1500)/500*PI); // To make sure this conversion is valid, servo_position should be in range [1100, 1900];
    servo_sin = sin((servo_position-1500)/500*PI);

    Rot_n2h << servo_cos, servo_sin, 0,
	      -servo_sin, servo_cos, 0,
    	       0,         0,         1;

    Rot_r2w = Rot_h2w * Rot_n2h * Rot_r2n;
//    std::cout << Rot_r2w << "\n" << std::endl;
    Rot_w2r = Rot_r2w.transpose();

    ocam2wpt = Rot_w2r * (waypoint - ocam_world);
    ocam2wpt2D(0) = ocam2wpt(0);
    ocam2wpt2D(1) = ocam2wpt(1);
    if (ocam2wpt(0) > 0)
    {  wptError(0) = ocam2wpt2D.norm();  }
    else
    {  wptError(0) = -ocam2wpt2D.norm();  }
    
    wptError(1) = atan2(ocam2wpt[1], ocam2wpt[0]);

    return wptError;
}

void stopBase()
{
      u_msg.angular.z = 0;
      u_msg.linear.x = 0;
      base_pub.publish(u_msg);
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
   close(fd); 
}

void BaseBackUp()
{
    u_msg.linear.x = flag_forward_backward * (-1) * 0.1;
    u_msg.angular.z = 0;
    base_pub.publish(u_msg);
    ros::Duration(2.0).sleep();
    stopBase(); 
}

void getServoPosition()
{
    int fd = maestroConnect();
    servo_position = maestroGetPosition(fd, 0)/4;
    close(fd);
}

bool checkBaseBoundary()
{
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    servo_center_world = ohand_world + Rot_h2w * ohand2servo_hand;  // Servo position in the world;

    getServoPosition();

    if (servo_position < 1000 || servo_position > 2000)
    {
	std::cout << "Warning! Invalid servo position: " << servo_position << " when adjusting Base!" << std::endl;
    }

    servo_cos = cos((servo_position-1500)/500*PI); // To make sure this conversion is valid, servo_position should be in range [1100, 1900];
    servo_sin = sin((servo_position-1500)/500*PI);

    Rot_n2h << servo_cos, servo_sin, 0,
	      -servo_sin, servo_cos, 0,
    	       0,         0,         1;

    Rot_r2w = Rot_h2w * Rot_n2h * Rot_r2n;

    base_center_world = servo_center_world - 0.0 * Rot_r2w.col(2);

    std::cout << "Base center in the world is: " << base_center_world(0) << " " << base_center_world(1) << " " <<  base_center_world(2) << 
                 " Servo center in the world is " << servo_center_world(0) << " " << servo_center_world(1) << " " << servo_center_world(2) << std::endl;

    bctr2ocam = ocam_world - base_center_world; 
    bctr2ocam2D(0) = bctr2ocam(0); bctr2ocam2D(1) = bctr2ocam(1);
    bctr2wpt = waypoint - base_center_world;
    bctr2wpt2D(0) = bctr2wpt(0); bctr2wpt2D(1) = bctr2wpt(1);

    std::cout << "Distance bctr2ocam: " << bctr2ocam2D.norm() << " , distance bctr2wpt: " << bctr2wpt2D.norm() << std::endl;

    if (bctr2wpt2D.norm() < 1.1 * bctr2ocam2D.norm())
    { return true; }
    else
    { return false; }
}

void ifWptclosetoBase()
{
    while (checkBaseBoundary())
    {
      std::cout << "Waypoint is too close to the base center!" << std::endl;
      BaseBackUp();
      ros::Duration(0.5).sleep();
    }
}

