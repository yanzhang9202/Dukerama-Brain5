#include "fstream"
#include "iostream"

#include "FlyCapture2.h"

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#define SERIAL_LEFT 13200942
#define SERIAL_RIGHT 13200944

using namespace FlyCapture2;

double x,y,z,a11,a12,a13,a21,a22,a23,a31,a32,a33;
bool updated;
bool left;

void PrintError( Error error )
{
    error.PrintErrorTrace();
}

void update_pos(const geometry_msgs::TransformStamped msg){
  x=msg.transform.translation.x;
  y=msg.transform.translation.y;
  z=msg.transform.translation.z;
  double a=msg.transform.rotation.w;
  double d=msg.transform.rotation.z;
  double c=msg.transform.rotation.y;
  double b=msg.transform.rotation.x;
  a11=1-2*b*b-2*c*c;//a*a+b*b-c*c-d*d;
  a12=2*a*b-2*c*d;//2*b*c-2*a*d;
  a13=2*a*c+2*b*d;//2*b*d+2*a*c;
  a21=2*a*b+2*c*d;//2*b*c+2*a*d;
  a22=1-2*a*a-2*c*c;//a*a-b*b+c*c-d*d;
  a23=2*b*c-2*a*d;//2*c*d-2*a*b;
  a31=2*a*c-2*b*d;//2*b*d-2*a*c;
  a32=2*b*c+2*a*d;//2*c*d+2*a*b;
  a33=1-2*a*a-2*b*b;//a*a-b*b-c*c+d*d;
  updated=true;
  return;
}

void print_to_file()
{
  std::ofstream myfile("/home/dukerama/Documents/hand_eye_calib/vrpn_data_right.txt", std::ios_base::app);
 
  if (!myfile.is_open()){
    std::cout << "Couldn't open file!" << std::endl;
  } else {
    myfile << a11 << " " << a12 << " " << a13 << " " << x << " \n";
    myfile << a21 << " " << a22 << " " << a23 << " " << y << " \n";
    myfile << a31<< " " << a32 << " " <<a33 << " " << z<< " \n";
    myfile << "0 0 0 1 \n";
  }
  myfile.close();
  return;
}

int main(int argc, char** argv)
{  
  left=false;
  
  ros::init(argc, argv, "HandEyeCalib");
  ros::NodeHandle n;

  //n.param<bool>("/HandEyeCalib/left",left,true);
  
  updated=false;

  ros::Subscriber vrpn_track=n.subscribe("/Brain5/pose",1000,update_pos);

  ros::Rate loop_rate(30);
  const Mode k_fmt7Mode = MODE_0;
  const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_RGB8;

    Error error;
    
    // Since this application saves images in the current folder
    // we must ensure that we have permission to write to this folder.
    // If we do not have permission, fail right away.
	FILE* tempFile = fopen("/home/dukerama/Documents/hand_eye_calib/test.txt", "w+");
	if (tempFile == NULL)
	{
		printf("Failed to create file in current folder.  Please check permissions.\n");
		return -1;
	}
	fclose(tempFile);
	remove("test.txt");

    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    printf( " The Number of cameras detected: %u\n", numCameras );

    if ( numCameras < 1 )
    {
        printf( "Insufficient number of cameras... exiting\n" );
        return -1;
    }

    PGRGuid guid_left;
    PGRGuid guid_right;
    error = busMgr.GetCameraFromSerialNumber(SERIAL_LEFT, &guid_left);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    error = busMgr.GetCameraFromSerialNumber(SERIAL_RIGHT, &guid_right);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    Camera cam_left;
    Camera cam_right;

    // Connect to a camera
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

    // Query for available Format 7 modes
    Format7Info fmt7Info;
    bool supported;
    fmt7Info.mode = k_fmt7Mode;
    error = cam_left.GetFormat7Info( &fmt7Info, &supported );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    if ( (k_fmt7PixFmt & fmt7Info.pixelFormatBitField) == 0 )
    {
      // Pixel format not supported!
      printf("Pixel format is not supported\n");
      return -1;
    }
    
    Format7ImageSettings fmt7ImageSettings;
    fmt7ImageSettings.mode = k_fmt7Mode;
    fmt7ImageSettings.offsetX = 2;
    fmt7ImageSettings.offsetY = 2;
    fmt7ImageSettings.width = 1276;
    fmt7ImageSettings.height = 1022;
    fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;
    
    bool valid;
    Format7PacketInfo fmt7PacketInfo;

    // Validate the settings to make sure that they are valid
    error = cam_left.ValidateFormat7Settings(
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
    error = cam_left.SetFormat7Configuration(
					     &fmt7ImageSettings,
					     fmt7PacketInfo.recommendedBytesPerPacket );
    if (error != PGRERROR_OK)
      {
        PrintError( error );
        return -1;
      }
    
    error = cam_right.SetFormat7Configuration(
        &fmt7ImageSettings,
        fmt7PacketInfo.recommendedBytesPerPacket );
    if (error != PGRERROR_OK)
      {
        PrintError( error );
        return -1;
      }
    // Start capturing images
    error = cam_left.StartCapture();
    if (error != PGRERROR_OK)
    {
      PrintError( error );
      return -1;
    }
    
    error = cam_right.StartCapture();
    if (error != PGRERROR_OK)
      {
        PrintError( error );
        return -1;
      }
    
    // Retrieve frame rate property
    Property frmRate;
    frmRate.type = FRAME_RATE;
    error = cam_left.GetProperty( &frmRate );
    if (error != PGRERROR_OK)
    {
      PrintError( error );
      return -1;
    }
    
    printf( "Frame rate is %3.2f fps and %u\n", frmRate.absValue, fmt7PacketInfo.recommendedBytesPerPacket );
    
    printf( "I am Grabbing images\n");
    
    int count=1;
    Image rawImage_left;   
    Image rawImage_right;
    
    while (ros::ok())
      {
	if (!updated){
	  ros::spinOnce();
	  //std::cout << "in loop ?!" << std::endl;
	  continue;
	}
	
	std::cout << "Begin!" << std::endl;
	if (left){
	  // Retrieve an image
        error = cam_left.RetrieveBuffer( &rawImage_left );
        if (error != PGRERROR_OK)
	  {
            PrintError( error );
            continue;
	  }
	
	ros::spinOnce();
	}
	else {
	  error = cam_right.RetrieveBuffer( &rawImage_right );
	  if (error != PGRERROR_OK)
	    {
	      PrintError( error );
	      continue;
	    }
	  ros::spinOnce();
	}
	printf( "." );
	
        // Get the raw image dimensions
        PixelFormat pixFormat;
        unsigned int rows, cols, stride;
	if (left){
	  rawImage_left.GetDimensions( &rows, &cols, &stride, &pixFormat );
	}
	else {
	  rawImage_right.GetDimensions( &rows, &cols, &stride, &pixFormat );
	}
	
        // Create a converted image
        Image convertedImage_left;
	Image convertedImage_right;
	
        // Convert the raw image
	if(left){
	  error = rawImage_left.Convert( PIXEL_FORMAT_BGRU, &convertedImage_left );
	  if (error != PGRERROR_OK)
	    {
	      PrintError( error );
	      return -1;
	    }  
	}
	else {
	  error = rawImage_right.Convert( PIXEL_FORMAT_BGRU, &convertedImage_right );
	  if (error != PGRERROR_OK)
	    {
	      PrintError( error );
	      return -1;
	    }  
	}
        // Create a unique filename
        char filename_left[512];
        sprintf( filename_left, "/home/dukerama/Documents/hand_eye_calib/left%d.bmp", count );
	
	char filename_right[512];
	  sprintf( filename_right, "/home/dukerama/Documents/hand_eye_calib/right%d.bmp", count );
	  
        // Save the image. If a file format is not passed in, then the file
        // extension is parsed to attempt to determine the file format.
	  if(left){
	    error = convertedImage_left.Save( filename_left );
	    if (error != PGRERROR_OK)
	      {
		PrintError( error );
		return -1;
	      }        
	  } else {
	    
	    error = convertedImage_right.Save( filename_right );
	    if (error != PGRERROR_OK)
	      {
		PrintError( error );
		return -1;
	      }  
	  }
	  print_to_file();
	  
	  printf("Count=%d\nPress enter to take another photo\n", count);
	  getchar();
	  count++;
	  
      }
    
    printf( "\nFinished grabbing images\n" );

    // Stop capturing images
    error = cam_left.StopCapture();
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }      

    error = cam_right.StopCapture();
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }      

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
    printf( "Done! Press Enter to exit...\n" );
    getchar();

	return 0;
}
