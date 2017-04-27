#include <pgrcamera_driver/TakePicAction.h>
#include <actionlib/server/simple_action_server.h>
#include <FlyCapture2.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#define SERIAL_LEFT 13200942
#define SERIAL_RIGHT 13200944

using namespace FlyCapture2;

typedef actionlib::SimpleActionServer<pgrcamera_driver::TakePicAction> Server;

uint numCameras;
Camera** cameras;
image_transport::CameraPublisher left_pub;
image_transport::CameraPublisher right_pub;
sensor_msgs::CameraInfo cam_left_info;
sensor_msgs::CameraInfo cam_right_info;

inline void PrintError( Error error ) {
  error.PrintErrorTrace();
}

void execute(const pgrcamera_driver::TakePicGoalConstPtr& goal, Server* as)
{
  Error error;
  
  sensor_msgs::Image image_left, image_right;
   
  for (uint i=0;i<numCameras;i++){
    Image pic;
    error=cameras[i]->RetrieveBuffer(&pic);
    ros::Time time=ros::Time::now();
    if (error!=PGRERROR_OK)
      {
	PrintError(error);
	return;
      }

    sensor_msgs::Image ros_image;
    sensor_msgs::fillImage (ros_image, sensor_msgs::image_encodings::RGB8,uint32_t( pic.GetRows()),uint32_t(pic.GetCols()),uint32_t(pic.GetStride()),pic.GetData());
    /*sensor_msgs::CameraInfo cam_info;
    ros_image.header.stamp=time;
    cam_info.header.stamp=time;*/
    
    ros_image.header.stamp=time;
    if (i==0)
      {
	image_left=ros_image;
	cam_left_info.header.stamp=time;
	//cam_info_left=cam_info;
      } 
    else if (i==1)
      {
	image_right=ros_image;
	cam_right_info.header.stamp=time;
	//cam_info_right=cam_info;
      }
  }

  left_pub.publish(image_left,cam_left_info);
  right_pub.publish(image_right,cam_right_info);
  
  pgrcamera_driver::TakePicResult result;

  result.finished=true;

  as->setSucceeded(result);
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

  int serials[2]={SERIAL_LEFT,SERIAL_RIGHT};
  
  for (unsigned int i=0; i<numCameras;i++)
    {
      cameras[i]=new Camera();
      PGRGuid guid;
      error=busMgr.GetCameraFromSerialNumber(serials[i],&guid);
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
  image_transport::ImageTransport it(n);
  left_pub = it.advertiseCamera("/stereo/left/image_raw", 1);
  right_pub=it.advertiseCamera("/stereo/right/image_raw", 1);

  std::string left_url, right_url;
  n.getParam("/take_pic_server/left_cam_url", left_url);
  std::cout << left_url << std::endl;
  n.getParam("/take_pic_server/right_cam_url", right_url);
  std::cout << right_url << std::endl;
  
  std::stringstream ss;

  /*  PGRGuid guid;
  busMgr.GetCameraFromIndex(0,&guid);
  std::cout << guid.value << std::endl;
  ss<<guid.value;*/
  camera_info_manager::CameraInfoManager cleft_info(n);
  //cleft_info=new camera_info_manager::CameraInfoManager(n);

  cleft_info.setCameraName("stereo/left");
  if (cleft_info.validateURL(left_url)){
    cleft_info.loadCameraInfo(left_url);
    cam_left_info=cleft_info.getCameraInfo();
    //  cam_left_info.header.frame_id
  } else {
    std::cout << "Left URL is invalid\n" << std::endl;
    return 0;
  }
    
  //camera_info_manager::CameraInfoManager cright_info(n);
  //cright_info=new camera_info_manager::CameraInfoManager(n);
  cleft_info.setCameraName("stereo/right");
  if (cleft_info.validateURL(right_url)){
    cleft_info.loadCameraInfo(right_url);
    cam_right_info=cleft_info.getCameraInfo();
  } else {
    std::cout << "Right URL is invalid\n" << std::endl;
    return 0;
  }
    
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
  
