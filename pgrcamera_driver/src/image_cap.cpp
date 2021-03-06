#include <pgrcamera_driver/TakePicAction.h>
#include <actionlib/server/simple_action_server.h>
#include <FlyCapture2.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <iostream>

using namespace FlyCapture2;

typedef actionlib::SimpleActionServer<pgrcamera_driver::TakePicAction> Server;

uint numCameras;
Camera** cameras;

inline void PrintError( Error error ) {
  error.PrintErrorTrace();
}

void execute(const pgrcamera_driver::TakePicGoalConstPtr& goal, Server* as)
{
  Error error;
  
  sensor_msgs::Image image_left, image_right;
  sensor_msgs::CameraInfo cam_info_left, cam_info_right;
  
  for (uint i=0;i<numCameras;i++){
    Image pic;
    error=cameras[i]->RetrieveBuffer(&pic);
    if (error!=PGRERROR_OK)
      {
	PrintError(error);
	return;
      }

    sensor_msgs::Image ros_image;
    sensor_msgs::fillImage (ros_image, sensor_msgs::image_encodings::RGB8,uint32_t( pic.GetRows()),uint32_t(pic.GetCols()),uint32_t(pic.GetStride()),pic.GetData());
    TimeStamp timestamp;
    timestamp=pic.GetTimeStamp();
    sensor_msgs::CameraInfo cam_info;
    ros::Time time(timestamp.seconds,timestamp.microSeconds);
    ros_image.header.stamp=time;
    cam_info.header.stamp=time;
    cam_info.height=ros_image.height;
    cam_info.width=ros_image.width;
    
    if (i==0)
      {
      image_left=ros_image;
      cam_info_left=cam_info;
      } 
    else if (i==1)
      {
	image_right=ros_image;
	cam_info_right=cam_info;
      }
  }
  
  pgrcamera_driver::TakePicResult result;

  result.image_left=image_left;
  result.image_right=image_right;
  result.camera_info_left=cam_info_left;
  result.camera_info_right=cam_info_right;

  as->setSucceeded(result,"Picture\n");
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"take_pic_server");
  ros::NodeHandle n;

  Error error;
  BusManager busMgr;
  const Mode k_fmt7Mode = MODE_0;
  const PixelFormat k_fmt7PixFmt=PIXEL_FORMAT_RGB8;
  
  error = busMgr.GetNumOfCameras(&numCameras);
  if (error != PGRERROR_OK)
    {
      PrintError( error );
      return -1;
    }
  
  printf( "Number of cameras detected: %u\n", numCameras );
  
  if ( numCameras != 2 )
    {
      printf( "Insufficient number of cameras... exiting\n" );
      return -1;
    }
  
  Format7ImageSettings fmt7ImageSettings;
  fmt7ImageSettings.mode = k_fmt7Mode;
  fmt7ImageSettings.offsetX = 2;
  fmt7ImageSettings.offsetY = 2;
  fmt7ImageSettings.width = 1276;
  fmt7ImageSettings.height = 1022;
  fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;
  
  Property frame_rate;
  frame_rate.type=FRAME_RATE;
  frame_rate.absValue=50.0;
  
  cameras = new Camera*[numCameras];
  
  for (uint i=0; i<numCameras;i++)
    {
      cameras[i]=new Camera();
      PGRGuid guid;
      
      error=busMgr.GetCameraFromIndex(i,&guid);
      if (error!= PGRERROR_OK)
	{
	  PrintError(error);
	  return -1;
	}
      
      error=cameras[i]->Connect(&guid);
      if (error!=PGRERROR_OK)
	{
	  PrintError(error);
	  return -1;
	}
      bool valid;
      Format7PacketInfo fmt7PacketInfo;
      
      // Validate the settings to make sure that they are valid
      error = cameras[i]->ValidateFormat7Settings(
						  &fmt7ImageSettings,
						  &valid,
						  &fmt7PacketInfo );
      if (error != PGRERROR_OK)
	{
	  PrintError( error );
	  return -1;
	}
      
      if ( !valid )
	{
	  // Settings are not valid
	  printf("Format7 settings are not valid\n");
	  return -1;
	}
      
      // Set the settings to the camera
      error = cameras[i]->SetFormat7Configuration(
						  &fmt7ImageSettings,
						  fmt7PacketInfo.recommendedBytesPerPacket );
      if (error != PGRERROR_OK)
	{
	  PrintError( error );
	  return -1;
	}       
      
      error=cameras[i]->SetProperty(&frame_rate);
      
      if (error != PGRERROR_OK)
	{
	  PrintError( error );
	  return -1;
	}       
      error=cameras[i]->StartCapture(); //Remove later
      
      if (error!=PGRERROR_OK)
	{
	  PrintError(error);
	  return -1;
	}
    }
  
  /*error=Camera::StartSyncCapture(numCameras, (const Camera**)cameras );
  if (error != PGRERROR_OK)
    {
      PrintError( error );
      return -1;
      }*/     

  printf("Server ready\n");

  Server server(n, "take_pic", boost::bind(&execute, _1, &server), false);
  printf("Server start\n");
  server.start();
  ros::spin();
  
  for (uint i=0;i<numCameras;i++)
    {
      cameras[i]->StopCapture();
      cameras[i]->Disconnect();
      delete cameras[i];
    }
  delete [] cameras;

  printf("Stopping\n");
       
  return 0;
}
  
