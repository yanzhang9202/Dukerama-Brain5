// This code publishes robot base frame in real time, given the Rot_h2w, servo position, Rot_r2n transformations.
// The servo position can vary from 1100us to 1900us.
// Created by Yan Zhang, on Sep 10, 2015.

#include "h2rlibrary.h"

void h2rCallback(const geometry_msgs::TransformStamped msg);
void servoCallback(const geometry_msgs::Point msg);

Eigen::Vector3f ocam_hand, ohand_world, ocam_world;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hand2robot");
  ros::NodeHandle node; 
  ros::Rate loop_rate(100); // This is the frequency how fast the robot updates its position, it should be larger than the controller frequency.

  ros::Subscriber h2r_sub = node.subscribe("/Brain5Camera/pose", 1, h2rCallback);

  ros::Subscriber servo_sub = node.subscribe("servo_position", 1, servoCallback);

  ros::Publisher r_pub = node.advertise<geometry_msgs::TransformStamped>("/Brain5Robot/pose", 1);

  // MAIN LOOP
  while( ros::ok() )
  {
    ros::spinOnce();

    // Publish the robot pose;
    r_pub.publish(msg_r);

    loop_rate.sleep();
  }

  return 0;
}

void servoCallback(const geometry_msgs::Point msg)
{
  servo_position = msg.x;
}

void h2rCallback(const geometry_msgs::TransformStamped msg)
{
    q_h[0] =msg.transform.rotation.x;
    q_h[1] =msg.transform.rotation.y;
    q_h[2] =msg.transform.rotation.z;
    q_h[3] =msg.transform.rotation.w;
  
    Eigen::Quaternionf quat_h(q_h[3], q_h[0], q_h[1], q_h[2]);

    Rot_h2w = quat_h.matrix(); // Return the hand2world rotation matrix;

   // Initialize the Rot_r2n transformation matrix. This should be included in a class definition in the future!

//    Rot_r2n << 0.4540, -0.8910, 0,
//	       0.8910, 0.4540, 0,
//    	       0,      0,      1;

    Rot_r2n << -0.2097, -0.9776, 0,
	        0.9776, -0.2097, 0,
    	        0,      0,      1;

    servo_cos = cos((servo_position-1500)/500*PI); // To make sure this conversion is valid, servo_position should be in range [1100, 1900];
    servo_sin = sin((servo_position-1500)/500*PI);
    printf("cosine: %f...\n", servo_cos);
    printf("sin: %f...\n", servo_sin);

    Rot_n2h << servo_cos, servo_sin, 0,
	      -servo_sin, servo_cos, 0,
    	       0,         0,         1;

    Rot_r2w = Rot_h2w * Rot_n2h * Rot_r2n; // Get the robot2world rotation matrix.

    // Input the translation vector and quaternion of robot to msg to be published. Think about what type of msgs are appropriate to be published.

//    std::cout << Rot_r2w << "\n" << std::endl;

    aa_r = Rot_r2w;
    Eigen::Quaternionf quat_r(aa_r);
    
    q_r = quat_r.coeffs(); // returned in [x, y, z, w] order.

    msg_r.transform.rotation.x = q_r[0];
    msg_r.transform.rotation.y = q_r[1];
    msg_r.transform.rotation.z = q_r[2];
    msg_r.transform.rotation.w = q_r[3];

    // This is where the hand markers' center position should be replaced by stereo camera rig position;

//    ocam_hand << -0.0611257, -0.0234336, 0.0047033;
//    ohand_world(0) = msg.transform.translation.x;
//    ohand_world(1) = msg.transform.translation.y;
//    ohand_world(2) = msg.transform.translation.z;
//    ocam_world = ohand_world + Rot_h2w * ocam_hand;

//    msg_r.transform.translation.x = ocam_world(0);
//    msg_r.transform.translation.y = ocam_world(1);
//    msg_r.transform.translation.z = ocam_world(2); 

    msg_r.transform.translation.x = msg.transform.translation.x;
    msg_r.transform.translation.y = msg.transform.translation.y;
    msg_r.transform.translation.z = msg.transform.translation.z; 
}
