// Edited by Yan Zhang, 06/01/2015

#ifndef ___triangulationNode_cpp
#define ___triangulationNode_cpp

#include <ros/ros.h>
#include "stereo_image_subscriber.cpp"
#include "stereo_localisation.cpp"
#include "iostream"
#include "fstream"
//#include <nav_msgs/Odometry.h>
//#include <sensor_msgs/Imu.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include "vrpnNode.cpp"

//void getPosition(nav_msgs::Odometry::ConstPtr odom_in, nav_msgs::Odometry* odom_out){
//    *odom_out=*odom_in;
//}

//void getEkfPosition(geometry_msgs::PoseWithCovarianceStamped::ConstPtr odom_in, geometry_msgs::PoseWithCovarianceStamped* odom_out){
//  *odom_out=*odom_in;
//}

//void getGyro(sensor_msgs::Imu::ConstPtr imu_in, sensor_msgs::Imu* imu_out){
//  *imu_out=*imu_in;
//}

int main(int argc, char**argv){
  ros::init(argc, argv, "circle_detector");
  ros::NodeHandle n;
  ros::Rate loop_rate(120);

  stereo_localisation localiser(NUM_TARGETS);
  
  std::string trackedBot="Brain7";
  std::string trackedCameras="Brain5";
  
  //CameraTrackable* bot = new CameraTrackable();
  //CameraTrackable* cameras= new CameraTrackable();
  //vrpnNode vrpnBot(n,bot,trackedBot);
  //vrpnNode vrpnCameras(n,cameras,trackedCameras);
  //std::queue<CameraTrackable> orientations;
  //Eigen::Vector3d lastPosition;
  bool updated=false;
  bool photoTaken=false;
  
  stereo_image_subscriber image_subscriber(n);
  //nav_msgs::Odometry odom;
  //sensor_msgs::Imu imu;
  //geometry_msgs::PoseWithCovarianceStamped odom_ekf;

  //ros::Subscriber odom_tracker=n.subscribe<nav_msgs::Odometry>("/odom",1,boost::bind(&getPosition,_1,&odom));
  //ros::Subscriber gyro_tracker=n.subscribe<sensor_msgs::Imu>("/imu/data",1,boost::bind(&getGyro,_1,&imu));
  //ros::Subscriber odom_ekf_tracker=n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose_ekf/odom",1,boost::bind(&getEkfPosition,_1,&odom_ekf));
  bool debug_mode=false;
  int imageCount=0;
  //std::ofstream outfile("/home/dukerama/Documents/ML_Project/Triangulations.txt");
  double timeLast,timeNow;
  timeLast=ros::Time::now().toSec();
  //std::cout << "Triangulation Started" << std::endl;
  for (int i=3;i>0;i--){
    std::cout <<"Starting in: " << i << std::endl;
    ros::Duration(1.0).sleep();
  }
  while (ros::ok()){
    if (vrpnBot.isUpdated() && bot->getPosition()[0]!=0 && vrpnCameras.isUpdated() && cameras->getPosition()[0]!=0){
      //std::cout << "Taking image" << std::endl;
      image_subscriber.take_photos();
      timeNow=ros::Time::now().toSec();
      nav_msgs::Odometry odom_current=odom;
      geometry_msgs::PoseWithCovarianceStamped odom_ekf_current=odom_ekf;
      sensor_msgs::Imu imu_current=imu;
      Eigen::Vector3d botPos=bot->getPosition();
      Eigen::Matrix3d botRot=bot->getRotation();
      Eigen::Vector3d cameraPos=cameras->getPosition();
      Eigen::Matrix3d cameraRot=cameras->getRotation();
      if (!image_subscriber.left_image.empty() && !image_subscriber.right_image.empty()){
	//photoTaken=true;
	//std::cout << "Image received" << std::endl;
	/*double ** coords_temp=new double*[N];
	  for (int i=0;i<4;i++){
	  coords_temp[i]=new double[4];
	  }*/
	localiser.localise_targets(image_subscriber.left_image,image_subscriber.right_image);
	//bool failed=false;
	for (int i=0;i<NUM_TARGETS;i++){
	  if (localiser.coords[i*4]==-1 || localiser.coords[i*4+2]==-1){ /*|| localiser.coords[i*4]<localiser.coords[i*4+2]) {
									   failed=true;*/
	    std::cout<<"Target " << i << " not found" << std::endl;
	  }
	}
	//if (failed) continue;
	//imageCount++;
	if (imageCount%100==0)ROS_INFO("%d",imageCount);
	//coords.push(coords_temp);
	/*if (failed) {
	  localiser.coords.clear();
	  continue;
	  }*/
	/*for (int i=0;i<N;i++){
	  Eigen::Vector3d p;
	  double xl=localiser.coords[i*4];
	  double yl=localiser.coords[i*4+1];
	  double xr=localiser.coords[i*4+2];
	  double yr=localiser.coords[i*4+3];
	  double y=(yl+yr)/2;
	  p(0)=BASELINE*(xl+xr)/(2*(xl-xr));
	  p(1)=BASELINE*y/(xl-xr);
	  p(2)=BASELINE*FOCAL_LENGTH/(xl-xr);
	  Eigen::Vector3d pD=cameraRot.transpose()*(botPos-cameraPos);
	  double xlD=FOCAL_LENGTH*(pD(0)+BASELINE/2)/pD(2);
	  double xrD=FOCAL_LENGTH*(pD(0)-BASELINE/2)/pD(2);
	  double yD=FOCAL_LENGTH*pD(1)/pD(2);
	  printf("xl is: %f, xr is: %f, yl is: %f, yr is: %f\n",xl,xr,yl,yr);
	  printf("xld is: %f, xrd is: %f, yd is: %f\n",xlD,xrD,yD);
	  Eigen::Vector3d x=cameraRot*p+cameraPos;
	  std::cout <<"x is:\n"<< x << "\ntrue x is:\n" << botPos<< std::endl;
	  }*/
     	
	/*if (failed){
	  continue;
	  }*/
	
	imageCount++;
	
        outfile << (timeNow-timeLast) << " ";
        timeLast=timeNow;
	for (int i=0;i<NUM_TARGETS;i++){
	  for (int j=0;j<4;j++){
	    outfile << localiser.coords[4*i+j] << " ";
	  }
	}
	
	for (int i=0;i<3;i++){
          outfile << cameraPos(i) << " ";
        }
        for (int i=0;i<3;i++){
          for (int j=0;j<3;j++){
            outfile << cameraRot(i,j) << " ";
          }
        }
	
	outfile << odom_current.pose.pose.position.x << " " << odom_current.pose.pose.position.y << " " << odom_current.pose.pose.position.z << " " <<  odom_current.pose.pose.orientation.w << " " <<  odom_current.pose.pose.orientation.x << " "<<  odom_current.pose.pose.orientation.y << " " <<  odom_current.pose.pose.orientation.z << " " << odom_current.twist.twist.linear.x << " " << odom_current.twist.twist.linear.y << " " << odom_current.twist.twist.linear.z << " " << imu_current.angular_velocity.x << " " << imu_current.angular_velocity.y << " " << imu_current.angular_velocity.z << " "; //odom_current.twist.twist.angular.x << " " << odom_current.twist.twist.angular.y << " " << odom_current.twist.twist.angular.z << " ";
	outfile << odom_ekf_current.pose.pose.position.x << " " << odom_ekf_current.pose.pose.position.y << " " << odom_ekf_current.pose.pose.position.z << " " <<  odom_ekf_current.pose.pose.orientation.w << " " <<  odom_ekf_current.pose.pose.orientation.x << " "<<  odom_ekf_current.pose.pose.orientation.y << " " <<  odom_ekf_current.pose.pose.orientation.z << " ";

	for (int i=0;i<3;i++){
	  outfile << botPos(i) << " "; 
	}
	for (int i=0;i<3;i++){
	  for (int j=0;j<3;j++){
	    outfile << botRot(i,j) << " ";
	  }
	}

	localiser.coords.clear();
	outfile << "\n";
	lastPosition=bot->getPosition();
	updated=true;
      }
    }
    ros::spinOnce();
    if (imageCount>300) break;
    //loop_rate.sleep();
  } 
  //outfile.close();
  return 0;
}

#endif
