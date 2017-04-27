#include <string.h>
#include <math.h>
#include "sensor_msgs/fill_image.h"
#include "sensor_msgs/CameraInfo.h"
//#include "sensor_msgs/Img.msg"
#include "FlyCapture2.h"
#include <time.h>
#include <ros/ros.h>
#include <iostream>
#include <TakePic.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;
using namespace FlyCapture2;

typedef actionlib::SimpleActionClient<TakePic> Client;

inline void PrintError( Error error )
{
    error.PrintErrorTrace();
}

int main(int argc, char** argv)
{

  // Use Actionlib
    Error error;

    BusManager busMgr;

    unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    printf( "Number of cameras detected: %u\n", numCameras );

    if ( numCameras < 2 )
    {
        printf( "Insufficient number of cameras... exiting\n" );
        return -1;
    }

    PGRGuid guid_left;
    PGRGuid guid_right;

    error = busMgr.GetCameraFromIndex(0, &guid_left);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    error = busMgr.GetCameraFromIndex(1, &guid_right);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }
    cout << "Got cameras\n" << endl;

    Camera cam_left;
    Camera cam_right;

    //Connect to both cameras
    error = cam_left.Connect(&guid_left);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    error = cam_right.Connect(&guid_right);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    cout <<"Cameras connected\n" << endl;

    //Loop starts here
    while(1){

      Image image_left;
      Image image_right;

      ros::init(argc, argv, "take_pic_client");
      Client client("take_pic", true); // true -> don't need ros::spin()
      client.waitForServer();
      client.sendGoal(cam_left);
      client.waitForResult(ros::Duration(1.0));
  
      // if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      //  printf("Picture taken.");
      // printf("Current State: %s\n", client.getState().toString().c_str());
 
      image_left = getResult();
    }

    cout << "Disconnecting the camera\n" << endl;

    // Disconnect the camera
    error = cam_left.Disconnect();
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    error = cam_right.Disconnect();
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }


    ros::init(argc, argv, "cam_capturer");
    ros::NodeHandle n;

    ros::Publisher image_publish=n.advertise<sensor_msgs::Image>("image", 10);

    ros::Rate loop_rate(30);

    while(ros::ok()){
      cout << "ROS!\n" << endl;
      image_publish.publish(ros_image);
      ros::spinOnce();
      loop_rate.sleep();
    }   

	return 0;
}
