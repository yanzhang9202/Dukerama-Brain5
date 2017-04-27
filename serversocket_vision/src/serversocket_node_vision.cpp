//#include "ServerSocket.h"
//#include "SocketException.h"
#include "ServerSocket.cpp"
#include "SocketException.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "Tank.h"
#include <sstream>
#include <string>
#include <iostream>
#include <string>
//#include <fstream>

//new code
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include "image_lib.cpp"

using namespace cv;
using namespace std;

void imageThreshold(const Mat &imgOriginal, Mat &imgThresholded, int iLowH, int iHighH, int iLowS, 
		    int iHighS, int iLowV, int iHighV, int remove_small, int fill_small, int percent);
//new code


//std::ofstream outfile_uleft ("/home/dukerama/hydro_catkin/src/serversocket_vision/include/uleft_pixel.txt", std::ofstream::out);
//std::ofstream outfile_uright ("/home/dukerama/hydro_catkin/src/serversocket_vision/include/uright_pixel.txt", std::ofstream::out);




int main ( int argc, char **argv)
{
  ros::init(argc, argv, "snn_control_rb");
  ros::NodeHandle node;


  ros::Rate loop_rate(10);

  int count = 0;
  std::cout << "not running....\n";
///new code
  //// Create two windows to show image
	namedWindow("right",CV_WINDOW_NORMAL);
	namedWindow("left",CV_WINDOW_NORMAL);

	namedWindow("thre_right",CV_WINDOW_NORMAL);
	namedWindow("thre_left",CV_WINDOW_NORMAL);	

	//// Declare robot name, server name and image topics' names
	string robotname, server_name, topic_name_left, topic_name_right;
	node.getParam("/robot/name", robotname); 
	server_name = robotname+"/take_pic";
	topic_name_left = robotname+"/left/image_rect_color";		// Take raw images or rectified images
	topic_name_right = robotname+"/right/image_rect_color";

	pgr_camera camera(node, topic_name_left, topic_name_right, server_name);

	namedWindow("Control", CV_WINDOW_NORMAL);

	int iLowH = 0;
	int iHighH = 179;

	int iLowS = 0; 
	int iHighS = 255;

	int iLowV = 0;
	int iHighV = 255;

	int remove_small = 5;
	int fill_small = 5;

	int percent = 0;
        
        Size size(100,100);
        Mat dst_left;
        Mat dst_right;
         
	//Create trackbars in "Control" window
	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);

	cvCreateTrackbar("Remove small objects", "Control", &remove_small, 20); //Value (0 - 20)
	cvCreateTrackbar("Fill in small holes", "Control", &fill_small, 20);

	cvCreateTrackbar("Mask of height(%)", "Control", &percent, 100);

//	ros::Rate loop_rate(10);
        cout<<"start to check camera"<<endl;
	camera.checkCamera();
        cout<<"camera checked"<<endl;
	Mat image_left, image_right;
	Mat imgThre_left, imgThre_right; 
          
///new code
  try
    {
      // Create the socket
      ServerSocket server ( 3100 );
      cout<<"socket created"<<endl;
      while ( true )
	{

	  ServerSocket new_sock;
	  server.accept ( new_sock );
          cout<<"accepted socket"<<endl;

	  try
	    {
                  while(ros::ok())
                        {
                        // new code
                          camera.takeStereoImages(1);
		image_left = camera.image_left;    image_right = camera.image_right;

		imshow("left", image_left);
		imshow("right", image_right);
		waitKey(30);

		imageThreshold(image_left, imgThre_left, iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, remove_small, fill_small, percent);
		imageThreshold(image_right, imgThre_right, iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, remove_small, fill_small, percent);
                
                Mat src_left=imgThre_left;
                Mat src_right=imgThre_right;
                resize(src_left,dst_left,size); 
                resize(src_right,dst_right,size);  
                   
		imshow("thre_left", dst_left);
		imshow("thre_right", dst_right);
                
                //new code//
                //outfile_uleft << dst_left << "\n";
                //cout<<"saving images"<<endl;
	        //outfile_uright << dst_right << "\n";
                //new code//
                cout<<dst_left.size()<<endl;
                cout<<dst_right.size()<<endl; 
                 // new code
                waitKey(30);  
                std::string data;
                cout << "sending data\n"<<data;
		new_sock >> data;
		          new_sock << data;
                          // std::cout << "running....\n"<<data<<"\nExiting.\n";
                          std::cout << "hello running....\n"<<data;  
                          ros::ServiceClient client=node.serviceClient<irobot_create_2_1::Tank>("tank");
                          irobot_create_2_1::Tank srv;
                          std::cout << "start\n"; 
                                 
                          std::string::size_type sz; 
                          double left=std::stod(data,&sz);
                          double right=std::stod(data.substr(sz)); 
                          
                          srv.request.left=left;
                          srv.request.right=right;
                          srv.request.clear=1;
                          if(client.call(srv))
                          {
                          ROS_INFO("Service call %d %d",srv.request.left,srv.request.right);
                          if(srv.response.success)
                          {ROS_INFO("Response");}
                          else
                          {ROS_INFO("No Response");}
                          }
                          else
                          {
                           ROS_INFO("Service NOT called");
                          }

                          ros::spinOnce();
                          loop_rate.sleep();
                          ++count;
                                                                       
                        }  
                                   
		
	    }
	  catch ( SocketException& ) {}

	}
    }
  catch ( SocketException& e )
    {
      std::cout << "Exception was caught:" << e.description() << "\nExiting.\n";
    }

  return 0;
}

///new code
void imageThreshold(const Mat &imgOriginal, Mat &imgThresholded, int iLowH, int iHighH, int iLowS,
		    int iHighS, int iLowV, int iHighV, int remove_small, int fill_small, int percent){
	Mat imgHSV;

	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
	      
	//morphological opening (remove small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(remove_small, remove_small)) );
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(remove_small, remove_small)) ); 

	//morphological closing (fill small holes in the foreground)
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(fill_small, fill_small)) ); 
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(fill_small, fill_small)) );

	cv::Size size = imgOriginal.size();
	cv::Mat mask = cv::Mat::zeros(size.height, size.width, CV_8U);
	mask(cv::Rect(0, percent*size.height/100, size.width, size.height-percent*size.height/100)) = 1;
	cv::Mat img_mask;
	imgThresholded.copyTo(img_mask, mask);
	imgThresholded = img_mask;
}

///new code


