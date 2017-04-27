#ifndef __LIB_ROBOT_CPP____
#define __LIB_ROBOT_CPP____

#include "lib_robot.h"

void waitForEnter(){
  char Enter;
  std::cin >> Enter;
}

void goWaypoints(){
  base_pid.clearhistory();
  wpt_dist_pid.clearhistory();
  wpt_ori_pid.clearhistory();

   while (robot_got_mission)
  {
      counter = 0;
      BaseError = getBaseError(); // unit: rad
      while(fabs(BaseError/PI*180) > Base_Orientation_Error_Tolerance && counter < 3)
      { 
        ros::spinOnce();
	if (flag_beat_in_transit)
	{ break; }

//        ifClosetoWpt();
	ifWptclosetoBase();
        if (flag_i_turned)
        { base_pid.setParam(1.0, 0.2, 0.1, 0.5, -0.5, PI/180); }

        adjustBase();
 
        if (flag_i_turned)
        {
          base_pid.setParam(1.0, 0.2, 0.1, 0.3, -0.3, Base_Orientation_Error_Tolerance/180*PI);
          flag_i_turned = false;
        }

        adjustBase();
 
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
	    counter = 0;
            break;
          }
//          ros::Duration(0.5).sleep();
        }      

        BaseError = getBaseError();
        counter++;
      }

        goingWaypoint(6*Waypoint_Error_Tolerance);

// *************************** Second Time ******************************* // 
      counter = 0;
      BaseError = getBaseError(); // unit: rad
      while(fabs(BaseError/PI*180) > Base_Orientation_Error_Tolerance && counter < 3)
      {
        ros::spinOnce();
	if (flag_beat_in_transit)
	{ break; }

//        ifClosetoWpt();
	ifWptclosetoBase();
        if (flag_i_turned)
        { base_pid.setParam(1.0, 0.2, 0.1, 0.5, -0.5, PI/180); counter = 0; }

        adjustBase();
 
        if (flag_i_turned)
        {
          base_pid.setParam(1.0, 0.2, 0.1, 0.3, -0.3, Base_Orientation_Error_Tolerance/180*PI);
          flag_i_turned = false;
        }

        adjustBase();

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
	    counter = 0;
            break;
          }
//          ros::Duration(0.5).sleep();
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
        ros::spinOnce();
	if (flag_beat_in_transit)
	{ break; }

        duration = ros::Time::now().toSec() - start_time;
        if (duration > waitforServo)
        {
          sendServoSpeed(); 
          break;
        }
        compromisesendServoInput();
      }  

      if (!flag_beat_in_transit)
      {
        getServoError();
        std::cout << "Servo Error: " << error_angle << std::endl;

// ********************************* End ****************************// 
        ros::spinOnce();
        ros::spinOnce();
        std::cout << "ocam_world: " << ocam_world(0) << " " << ocam_world(1) << " " << ocam_world(2) << "\n"; 
        std::cout << "waypoint: " << waypoint(0) << " " << waypoint(1) << " " << waypoint(2) << "\n";
      }    
      robot_got_mission = false;
  }
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

    Eigen::AngleAxisf aa_cam_world;
    aa_cam_world = Rot_c2w;
    Eigen::Quaternionf quat_cam(aa_cam_world);
    q_cam = quat_cam.coeffs(); // returned in [x, y, z, w] order.

    flag_getCurrentPosition = 1;
  }
}

float getBaseError()
{
    Eigen::Vector3f waypoint3D;
    waypoint3D(0) = waypoint(0); waypoint3D(1) = waypoint(1); waypoint3D(2) = waypoint(2);

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

    ocam2wpt = Rot_w2r * (waypoint3D - ocam_world) * flag_forward_backward;
    //return (atan2(ocam2tgt[1], ocam2tgt[0])/PI*180);
//    std::cout << ocam2wpt(0) << " " << ocam2wpt(1) << " " << ocam2wpt(2) << "\n";
//    std::cout << ocam2wpt(0) << " " << ocam2wpt(1) << " " << ocam2wpt(2) << "\n";
    return (atan2(ocam2wpt[1], ocam2wpt[0]));
}

void ifClosetoWpt()
{
  Eigen::Vector3f waypoint3D;
  waypoint3D(0) = waypoint(0); waypoint3D(1) = waypoint(1); waypoint3D(2) = waypoint(2);

  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce(); 
  ocam2wpt_world = waypoint3D - ocam_world;
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

    Eigen::Vector3f waypoint3D;
    waypoint3D(0) = waypoint(0); waypoint3D(1) = waypoint(1); waypoint3D(2) = waypoint(2);
    tgt_world = target;

    wpt2tgt_robot = Rot_w2r * (tgt_world - waypoint3D);
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
        ros::spinOnce();
	if (flag_beat_in_transit)
	{ break; }

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

    Eigen::Vector3f waypoint3D;
    waypoint3D(0) = waypoint(0); waypoint3D(1) = waypoint(1); waypoint3D(2) = waypoint(2);
    tgt_world = target;

    std::cout << "Target is " << tgt_world(0) << " " << tgt_world(1) << " " << tgt_world(2) << std::endl;  

    wpt2tgt = Rot_w2c * (tgt_world - waypoint3D);
//    std::cout << tgt_world << std::endl;
//    std::cout << waypoint << std::endl;
//    std::cout << Rot_c2w << std::endl;
    error_angle = atan2(-wpt2tgt[0], wpt2tgt[2])/PI*180;
}

void compromisesendServoInput()
{
    getServoError();
    getServoPosition();

    while(servo_position < 1000 || servo_position > 2000)
    {
	std::cout << "Invalid servo position: " << servo_position << "! Request again!" << std::endl;
        sendServoSpeed();
        initialServoTarget();
	getServoError();
	ros::Duration(0.1).sleep();
    }

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
          { 
	     servo_target = servo_position*4 + servo_delta;
             int fd = maestroConnect();
             maestroSetTarget(fd, 0, servo_target);
             while(GetMovingState(fd,0)){}
             close(fd);
 	  }
          else
          { 
            std::cout << "Servo remains where it is!" << std::endl;
          }
        }
        else
        {
           int fd = maestroConnect();
           maestroSetTarget(fd, 0, servo_target);
           while(GetMovingState(fd,0)){}
           close(fd);
        }
     }

    if (error_angle > 0)
    {
         float servo_target = servo_position*4 + servo_delta;

         if ((servo_position + servo_delta/4)>1875)
         {
           error_angle = error_angle - 360;
           servo_delta = round(error_angle/180*2000);
           if ((servo_position + servo_delta/4 > 1125))
           { 
	      servo_target = servo_position*4 + servo_delta;
              int fd = maestroConnect();
              maestroSetTarget(fd, 0, servo_target);
              while(GetMovingState(fd,0)){}
              close(fd);
	   }
           else
           { 
              std::cout << "Servo remains where it is!" << std::endl;
           } 
        }
	else
	{
            int fd = maestroConnect();
            maestroSetTarget(fd, 0, servo_target);
            while(GetMovingState(fd,0)){}
            close(fd);
	}
    }
  }
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

    Eigen::Vector3f waypoint3D;
    waypoint3D(0) = waypoint(0); waypoint3D(1) = waypoint(1); waypoint3D(2) = waypoint(2);

    ocam2wpt = Rot_w2r * (waypoint3D - ocam_world);
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
   maestroSetSpeed(fd, 0, 3);
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
    ros::Duration(0.5).sleep();
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

    base_center_world = servo_center_world - 0.0 * Rot_r2w.col(1);

    std::cout << "Base center in the world is: " << base_center_world(0) << " " << base_center_world(1) << " " <<  base_center_world(2) << 
                 " Servo center in the world is " << servo_center_world(0) << " " << servo_center_world(1) << " " << servo_center_world(2) << std::endl;

    Eigen::Vector3f waypoint3D;
    waypoint3D(0) = waypoint(0); waypoint3D(1) = waypoint(1); waypoint3D(2) = waypoint(2);

    bctr2ocam = ocam_world - base_center_world; 
    bctr2ocam2D(0) = bctr2ocam(0); bctr2ocam2D(1) = bctr2ocam(1);
    bctr2wpt = waypoint3D - base_center_world;
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

void waitForServoStill()
{
    int fd = maestroConnect();
    while(GetMovingState(fd,0)){ std::cout << "Servo is moving!" << std::endl; ros::Duration(1.0).sleep(); }
    close(fd);
}

#endif
