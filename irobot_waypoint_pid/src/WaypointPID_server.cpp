// This node plays as a server for a client in "irobot_waypoint_brain" package, "irobot_waypoint_brain" node.
// Created by Yan Zhang, on September 12nd, 2015.
// Please refer to http://wiki.ros.org/pid for pid controller details.

#include "pid_header.h"

#include <irobot_waypoint_pid/WaypointPIDAction.h>
#include <actionlib/server/simple_action_server.h>

#define ERROR_TOLERANCE 0.020

geometry_msgs::Point goal_msg;

ros::Publisher pid_pub;
ros::Subscriber state_sub;
ros::Subscriber servo_claim_control_sub;

typedef actionlib::SimpleActionServer<irobot_waypoint_pid::WaypointPIDAction> Server;

void servoclaimCallback(const geometry_msgs::Point msg);
int servo_claim_control = 0;

void execute(const irobot_waypoint_pid::WaypointPIDGoalConstPtr& goal, Server* as)
{ 
  goal_msg = goal->waypoint;
  waypoint(0) = goal_msg.x;
  waypoint(1) = goal_msg.y;
  waypoint(2) = goal_msg.z;

  //std::cout << "i heard " << waypoint << std::endl;

  // Initialize the parmas in pid
  prev_time = 0.0;
  delta_t = 0.0;
  error_integral_speed = 0.0;
  error_integral_head = 0.0;

  error_speed.setZero();
  filtered_error_speed.setZero();
  error_deriv_speed.setZero();
  filtered_error_deriv_speed.setZero();
  loop_counter_speed = 0;

  error_head.setZero();
  filtered_error_head.setZero();
  error_deriv_head.setZero();
  filtered_error_deriv_head.setZero();
  loop_counter_head = 0;

  mission_flag = true;

  while(mission_flag)
  {
    ros::spinOnce();

    if(!isnan(u_msg.linear.x) && !isnan(u_msg.angular.z))  
    {
      pid_pub.publish(u_msg);
    }
  }

  as -> setSucceeded();
}

int main(int argc, char **argv)
{
  // Initialize ROS stuff
  ros::init(argc, argv, "WaypointPID_server");
  ros::NodeHandle node;

  // Publish on "/cmd_vel" topic
  pid_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  // Subscribe to robot pose topic
  state_sub = node.subscribe("/Brain5Robot/pose", 1, robotposeCallback );
  servo_claim_control_sub = node.subscribe("servo/claim_control", 1, servoclaimCallback);

  Server server(node, "WaypointPID_server", boost::bind(&execute, _1, &server), false);
  server.start();

  ros::spin();

  return 0;
}

// Callback for state update from VRPN;

void robotposeCallback(const geometry_msgs::TransformStamped msg)
{
  if(mission_flag && (servo_claim_control == 0))
  {
    // Calculate delta_t for numerical integral and derivative
    if (prev_time == 0)
    {prev_time = ros::Time::now().toSec();}
    else
    {
      delta_t = ros::Time::now().toSec() - prev_time;
      prev_time = ros::Time::now().toSec();
    }

    // load current robot position and heading message
    position_r(0) = msg.transform.translation.x;
    position_r(1) = msg.transform.translation.y;
    position_r(2) = msg.transform.translation.z;

    q_r[0] =msg.transform.rotation.x;
    q_r[1] =msg.transform.rotation.y;
    q_r[2] =msg.transform.rotation.z;
    q_r[3] =msg.transform.rotation.w;

    Eigen::Quaternionf quat_r(q_r[3], q_r[0], q_r[1], q_r[2]);
    Rot_r2w = quat_r.matrix();
    Rot_w2r = Rot_r2w.transpose();

// Test the Rot_r2w here, by moving the robot head forward and see if the x axis of Rot_w2r is correct.
//    std::cout << Rot_w2r << "\n" << std::endl;

    waypoint_r = Rot_w2r * (waypoint - position_r);
// Test the waypoint is correctly transformer into robot's frame;
//    std::cout << waypoint_r << "\n" << std::endl;

    waypoint_r2D(0) = waypoint_r(0);
    waypoint_r2D(1) = waypoint_r(1); 

// Speed PID controller

    error_speed(0) = waypoint_r2D(0); // Error as input to speed pid controller; 

    error_speed(2) = error_speed(1);
    error_speed(1) = error_speed(0);
  
//    std::cout << delta_t << "\n" << std::endl;

    if (abs(error_speed(0)) < ERROR_TOLERANCE) // The robot center is within 10mm range of waypoint
    {
       u_msg.linear.x = 0;
    }
    else
    {
      // integrate the error
      error_integral_speed += error_speed(0)*delta_t;

      // Apply anti-windup to limit the size of the integral term
      if ( error_integral_speed > abs( anti_w ) )
        error_integral_speed = abs(anti_w);

      if ( error_integral_speed < -abs( anti_w ) )
        error_integral_speed = -abs(anti_w);

      // My filter reference was Julius O. Smith III, Intro. to Digital Filters With Audio Applications.
      float c;
      if (cutoff_frequency == -1)
        c = 1.0; // Default to a cut-off frequency at one-fourth of the sampling rate
      else
        c = 1/tan( (cutoff_frequency*6.2832)*delta_t/2 );
 
      filtered_error_speed(2) = filtered_error_speed(1);
      filtered_error_speed(1) = filtered_error_speed(0); 
      filtered_error_speed(0) = (1/(1+c*c+1.414*c))*(error_speed(2)+2*error_speed(1)+error_speed(0)-(2-1.414)*filtered_error_speed(2));

      // Take derivative of error
      // First the raw, unfiltered data:
      error_deriv_speed(2) = error_deriv_speed(1);
      error_deriv_speed(1) = error_deriv_speed(0);
      error_deriv_speed(0) = (error_speed(0)-error_speed(1))/delta_t;

      filtered_error_deriv_speed(2) = filtered_error_deriv_speed(1);
      filtered_error_deriv_speed(1) = filtered_error_deriv_speed(0);

      if ( loop_counter_speed > 2 ) // Let some data accumulate
        filtered_error_deriv_speed(0) = (1/(1+c*c+1.414*c))*(error_deriv_speed(2)+2*error_deriv_speed(1)+error_deriv_speed(0)-(2-1.414)*filtered_error_deriv_speed(2));
      else
        loop_counter_speed++;

      // calculate the control effort
      u_msg.linear.x = Kp_speed * filtered_error_speed(0)+Ki_speed * error_integral_speed+ Kd_speed * filtered_error_deriv_speed(0);

      // Check saturation limits  
      if (u_msg.linear.x > ul_speed)
        u_msg.linear.x = ul_speed;
      if (u_msg.linear.x < ll_speed)
        u_msg.linear.x = ll_speed;
    }

// Heading PID controller

    if (waypoint_r2D(0) == 0){waypoint_r2D(0) = 0.001;} 
    error_head(0) = waypoint_r2D(1) / waypoint_r2D(0); // Error as input to heading pid controller;
    //std::cout << error_head(0) << "\n" << std::endl;

    error_head(2) = error_head(1);
    error_head(1) = error_head(0);
  
//    std::cout << delta_t << "\n" << std::endl;

    if (abs(error_head(0)) < ERROR_TOLERANCE) // The robot center is within 10mm range of waypoint
    {
       u_msg.angular.z = 0;
    }
    else
    {
      // integrate the error
      error_integral_head += error_head(0)*delta_t;

      // Apply anti-windup to limit the size of the integral term
      if ( error_integral_head > abs( anti_w ) )
        error_integral_head = abs(anti_w);

      if ( error_integral_head < -abs( anti_w ) )
        error_integral_head = -abs(anti_w);

      // My filter reference was Julius O. Smith III, Intro. to Digital Filters With Audio Applications.
      float c;
      if (cutoff_frequency == -1)
        c = 1.0; // Default to a cut-off frequency at one-fourth of the sampling rate
      else
        c = 1/tan( (cutoff_frequency*6.2832)*delta_t/2 );
 
      filtered_error_head(2) = filtered_error_head(1);
      filtered_error_head(1) = filtered_error_head(0); 
      filtered_error_head(0) = (1/(1+c*c+1.414*c))*(error_head(2)+2*error_head(1)+error_head(0)-(2-1.414)*filtered_error_head(2));

      // Take derivative of error
      // First the raw, unfiltered data:
      error_deriv_head(2) = error_deriv_head(1);
      error_deriv_head(1) = error_deriv_head(0);
      error_deriv_head(0) = (error_head(0)-error_head(1))/delta_t;

      filtered_error_deriv_head(2) = filtered_error_deriv_head(1);
      filtered_error_deriv_head(1) = filtered_error_deriv_head(0);

      if ( loop_counter_head > 2 ) // Let some data accumulate
        filtered_error_deriv_head(0) = (1/(1+c*c+1.414*c))*(error_deriv_head(2)+2*error_deriv_head(1)+error_deriv_head(0)-(2-1.414)*filtered_error_deriv_head(2));
      else
        loop_counter_head ++;

      // calculate the control effort
      u_msg.angular.z = Kp_head * filtered_error_head(0)+Ki_head * error_integral_head+ Kd_head * filtered_error_deriv_head(0);

      // Check saturation limits  
      if (u_msg.angular.z > ul_head)
        u_msg.angular.z = ul_head;
      if (u_msg.angular.z < ll_head)
        u_msg.angular.z = ll_head;
    }

    // See if the goal has been achieved
    if (abs(error_speed(0)) < ERROR_TOLERANCE)
    {
      mission_flag = false; u_msg.linear.x = 0; u_msg.angular.z = 0;
      printf("Waypoint [%f,%f,%f] achieved!\n", waypoint(0),waypoint(1),waypoint(2));
    }
  }
  else if (servo_claim_control == 1)// mission_flag = false
       {
         printf("Robot : Servo is controlling the base.\n");
         u_msg.linear.x = NAN; u_msg.angular.z = NAN;
       }
       else
       {
         u_msg.linear.x = NAN; u_msg.angular.z = NAN;
       }
}

void servoclaimCallback(const geometry_msgs::Point msg)
{
  servo_claim_control = msg.x;
}

