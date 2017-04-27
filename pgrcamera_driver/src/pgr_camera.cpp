#ifndef _PGRCAMERA_CPP
#define _PGRCAMERA_CPP

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "pgr_camera.h"

#define SERIAL_LEFT 13200942
#define SERIAL_RIGHT 13200944

//#define SERIAL_LEFT 13344924
//#define SERIAL_RIGHT 13344950

using namespace FlyCapture2;

/*sensor_msgs::Image pgr_camera::downsample(const sensor_msgs::Image& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");//sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return msg;
    }
    cv::Mat image=cv_ptr->image;
    cv::Mat image_out;
    cv::pyrDown(image,image_out,cv::Size((image.rows)/4,(image.cols)/4));
    return *cv_ptr->toImageMsg();
    }*/

//typedef actionlib::SimpleActionServer<pgrcamera_driver::TakePicAction> Server;
void pgr_camera::execute()
{
  pgrcamera_driver::TakePicGoal goal=*server.acceptNewGoal();
  sensor_msgs::Image image_left, image_right;
  
  Image pic_l, pic_r;
  //printf("Retrieving buffer 0 and 1\n");
  error=cameras[0]->RetrieveBuffer(&pic_l);
  error=cameras[1]->RetrieveBuffer(&pic_r);

  ros::Time time = ros::Time::now();
  
  if (error!=PGRERROR_OK)
    {
      PrintError(error);
      return;
    }
  //printf("Buffers 0 and 1 retreived\n");

  //sensor_msgs::Image ros_image;
  sensor_msgs::fillImage (image_left, sensor_msgs::image_encodings::RGB8,uint32_t( pic_l.GetRows()),uint32_t(pic_l.GetCols()),uint32_t(pic_l.GetStride()),pic_l.GetData());
  sensor_msgs::fillImage (image_right, sensor_msgs::image_encodings::RGB8,uint32_t( pic_r.GetRows()),uint32_t(pic_r.GetCols()),uint32_t(pic_r.GetStride()),pic_r.GetData());

  /*sensor_msgs::CameraInfo cam_info;
    ros_image.header.stamp=time;
    cam_info.header.stamp=time;*/
  image_left.header.stamp=time;
  image_right.header.stamp=time;
    
  cam_left_info.header.stamp=time;
  //cam_info_left=cam_info;
  
  cam_right_info.header.stamp=time;
  //cam_info_right=cam_info;
    
  /*if (goal.acquire) {
        image_left=downsample(image_left);
        image_right=downsample(image_right);
	}*/

  left_pub.publish(image_left,cam_left_info);
  right_pub.publish(image_right,cam_right_info);
  printf("Left and right published\n");
  
  /*pgrcamera_driver::TakePicResult result;
    result.finished=true;*/
  
  server.setSucceeded(result);
}

pgr_camera::pgr_camera(ros::NodeHandle n_priv, ros::NodeHandle n_left, ros::NodeHandle n_right):server(n_priv, "Brain5/take_pic",false),it_left(n_left),cam_info_left(n_left),it_right(n_right),cam_info_right(n_right)
//pgr_camera::pgr_camera(ros::NodeHandle n_priv, ros::NodeHandle n_left, ros::NodeHandle n_right):it_left(n_left),cam_info_left(n_left),it_right(n_right),cam_info_right(n_right)
{
  std::cout << "Initialised" << std::endl;

//  std::string robotname, temp, result;
//  n_priv.getParam("/robot/name", robotname);
//  temp = "/take_pic";
//  result = robotname + temp;
//  server(n_priv, result.c_str(), false);
  server.registerGoalCallback(boost::bind(&pgr_camera::execute,this));

  n_left.getParam("/brain5_camera_driver/left_cam_url", left_url);
  n_right.getParam("/brain5_camera_driver/right_cam_url", right_url);

  cam_info_left.setCameraName("left");
  if (cam_info_left.validateURL(left_url)){
    cam_info_left.loadCameraInfo(left_url);
    cam_left_info=cam_info_left.getCameraInfo();
  } else {
    std::cout << "Left URL is invalid\n" << std::endl;
    }
  
  cam_info_right.setCameraName("right");
  if (cam_info_right.validateURL(right_url)){
    cam_info_right.loadCameraInfo(right_url);
    cam_right_info=cam_info_right.getCameraInfo();
  } else {
    std::cout << "Right URL is invalid\n" << std::endl;
    }

  k_fmt7Mode = MODE_0;
  k_fmt7PixFmt=PIXEL_FORMAT_RGB8;
  result.finished=true;
}

inline void pgr_camera::PrintError( Error error ) {
  error.PrintErrorTrace();
}

int pgr_camera::connect_cameras(/*int serials[]={-1,-1}*/){
  int serials[]={SERIAL_LEFT,SERIAL_RIGHT};
  error = busMgr.GetNumOfCameras(&numCameras);
  if (error != PGRERROR_OK)
    {
      PrintError( error );
      return -1;
    }
  
  printf( "Number of cameras detected: %u\n", numCameras );
  
  if ( numCameras != 2 )
    {
      printf( "Insufficient number of cameras... Num cameras=%d... exiting\n", numCameras );
      return -1;
    }
  cameras= new Camera*[numCameras];
  for (unsigned int i=0; i<numCameras;i++)
    {
      cameras[i]=new Camera();
      printf("Camera %d created\n",i);
      PGRGuid guid;
      if (serials[i]!=-1){
	error=busMgr.GetCameraFromSerialNumber(serials[i],&guid);
	if (error!= PGRERROR_OK)
	  {
	    PrintError(error);
	    return -1;
	  }
      } else {
	error=busMgr.GetCameraFromIndex(i,&guid);
	if (error!= PGRERROR_OK)
	  {
	    PrintError(error);
	    return -1;
	  }
      }
      
      printf("Cameras gotten\n");
      error=cameras[i]->Connect(&guid);
      if (error!=PGRERROR_OK)
	{
	  PrintError(error);
	  return -1;
	}
      printf("Cameras connected\n");
    }
}

int pgr_camera::start_cameras(){
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
    
  bool valid;
  Format7PacketInfo fmt7PacketInfo;
    
  for (uint i=0;i<numCameras;i++){
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
    
    printf("Camera properties set\n");
  }
  StrobeControl mStrobe;
  printf("Strobecontrol created\n");
  mStrobe.source=2;
  mStrobe.delay=0;
  mStrobe.duration=0;
  mStrobe.onOff=true;
  mStrobe.polarity=1;
  error=cameras[0]->SetStrobe(&mStrobe);
  if (error != PGRERROR_OK)
    {
      PrintError( error);
      return -1;
    }
      
  printf("Camera strobe set\n");

  TriggerMode myTrigger;
  myTrigger.mode=0;
  myTrigger.onOff=true;
  myTrigger.parameter=0;
  myTrigger.polarity=1;
  myTrigger.source=3;
  error=cameras[1]->SetTriggerMode(&myTrigger);
  if (error != PGRERROR_OK)
    {
      PrintError( error);
      return -1;
    }
      
  printf("Camera Trigger set\n");

  /*error=Camera::StartSyncCapture(numCameras, (const Camera**)cameras );
  if (error != PGRERROR_OK)
    {
      PrintError( error );
      return -1;
    }
    printf("Sync capture started\n");*/
  for (int i=0;i<numCameras;i++){
  error=cameras[i]->StartCapture(); //Remove later
    
     if (error!=PGRERROR_OK)
     {
     PrintError(error);
     return -1;
    }
  }
}      

   

int pgr_camera::disconnect_cameras(){
  cameras[0]->StopCapture();
  cameras[1]->StopCapture();

  StrobeControl mStrobe;
  mStrobe.source=2;
  mStrobe.delay=0;
  mStrobe.duration=0;
  mStrobe.onOff=false;
  mStrobe.polarity=1;
  cameras[0]->SetStrobe(&mStrobe);
      
  TriggerMode myTrigger;
  myTrigger.mode=0;
  myTrigger.onOff=false;
  myTrigger.parameter=0;
  myTrigger.polarity=1;
  myTrigger.source=3;
  cameras[1]->SetTriggerMode(&myTrigger);

  for (uint i=0;i<numCameras;i++)
    {
      cameras[i]->Disconnect();
    }
        
  printf("Stopping\n");
}

int pgr_camera::start_stream(){
  left_pub = it_left.advertiseCamera("image_raw", 10);
  right_pub=it_right.advertiseCamera("image_raw", 10);

  printf("Server start\n");
  server.start();
  ros::spin();
    
  return 0;
}
  
#endif
