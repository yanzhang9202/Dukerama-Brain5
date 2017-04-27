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
#include <fstream>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <typeinfo>
//#include <fstream>

//new code
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>

#include "image_lib.cpp"

using namespace cv;
using namespace std;
/*
set global variable to be used to connect
button and trackbar
*/
int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, remove_small, fill_small, percent;
Mat3b canvas;
Rect buttonLowH, buttonHighH, buttonLowS, buttonHighS, buttonLowV, buttonHighV;

void imageThreshold(const Mat &imgOriginal, Mat &imgThresholded, int iLowH, int iHighH, int iLowS, 
		    int iHighS, int iLowV, int iHighV, int remove_small, int fill_small, int percent);
//new code

// Define callback functions for each trackbars
void callBackButton(int state, int x, int y, int flags, void* userdata)
{
	if(state == EVENT_LBUTTONDOWN)
	{
		// if LowH
		if(buttonLowH.contains(Point(x, y)))
		{
			if(flags == (EVENT_FLAG_ALTKEY + EVENT_FLAG_LBUTTON))
			{

				// change Trackbar position
				int g_frame_count = (int) cvGetTrackbarPos("LowH", "Control");
				cvSetTrackbarPos("LowH", "Control", ++g_frame_count);
			}

			if(flags == (EVENT_FLAG_CTRLKEY + EVENT_FLAG_LBUTTON))
			{

				// change Trackbar position
				int g_frame_count = (int) cvGetTrackbarPos("LowH", "Control");
				cvSetTrackbarPos("LowH", "Control", --g_frame_count);
			}
		}else if(buttonHighH.contains(Point(x, y)))
		{
			if(flags == (EVENT_FLAG_ALTKEY + EVENT_FLAG_LBUTTON))
			{
				// change Trackbar position
				int g_frame_count = (int) cvGetTrackbarPos("HighH", "Control");
				cvSetTrackbarPos("HighH", "Control", ++g_frame_count);
			}

			if(flags == (EVENT_FLAG_CTRLKEY + EVENT_FLAG_LBUTTON))
			{
				// change Trackbar position
				int g_frame_count = (int) cvGetTrackbarPos("HighH", "Control");
				cvSetTrackbarPos("HighH", "Control", --g_frame_count);
			}
		}else if(buttonLowS.contains(Point(x, y)))
		{
			if(flags == (EVENT_FLAG_ALTKEY + EVENT_FLAG_LBUTTON))
			{

				// change Trackbar position
				int g_frame_count = (int) cvGetTrackbarPos("LowS", "Control");
				cvSetTrackbarPos("LowS", "Control", ++g_frame_count);
			}

			if(flags == (EVENT_FLAG_CTRLKEY + EVENT_FLAG_LBUTTON))
			{

				// change Trackbar position
				int g_frame_count = (int) cvGetTrackbarPos("LowS", "Control");
				cvSetTrackbarPos("LowS", "Control", --g_frame_count);
			}
		}
		else if(buttonHighS.contains(Point(x, y)))
		{
			if(flags == (EVENT_FLAG_ALTKEY + EVENT_FLAG_LBUTTON))
			{

				// change Trackbar position
				int g_frame_count = (int) cvGetTrackbarPos("HighS", "Control");
				cvSetTrackbarPos("HighS", "Control", ++g_frame_count);
			}

			if(flags == (EVENT_FLAG_CTRLKEY + EVENT_FLAG_LBUTTON))
			{

				// change Trackbar position
				int g_frame_count = (int) cvGetTrackbarPos("HighS", "Control");
				cvSetTrackbarPos("HighS", "Control", --g_frame_count);
			}
		}
		else if(buttonLowV.contains(Point(x, y)))
		{
			if(flags == (EVENT_FLAG_ALTKEY + EVENT_FLAG_LBUTTON))
			{

				// change Trackbar position
				int g_frame_count = (int) cvGetTrackbarPos("LowV", "Control");
				cvSetTrackbarPos("LowV", "Control", ++g_frame_count);
			}

			if(flags == (EVENT_FLAG_CTRLKEY + EVENT_FLAG_LBUTTON))
			{

				// change Trackbar position
				int g_frame_count = (int) cvGetTrackbarPos("LowV", "Control");
				cvSetTrackbarPos("LowV", "Control", --g_frame_count);
			}
		}
		else if(buttonHighV.contains(Point(x, y)))
		{
			if(flags == (EVENT_FLAG_ALTKEY + EVENT_FLAG_LBUTTON))
			{

				// change Trackbar position
				int g_frame_count = (int) cvGetTrackbarPos("HighV", "Control");
				cvSetTrackbarPos("HighV", "Control", ++g_frame_count);
			}

			if(flags == (EVENT_FLAG_CTRLKEY + EVENT_FLAG_LBUTTON))
			{
				// change Trackbar position
				int g_frame_count = (int) cvGetTrackbarPos("HighV", "Control");
				cvSetTrackbarPos("HighV", "Control", --g_frame_count);
			}
		}
	}
	imshow("Control", canvas);
}

//std::ofstream outfile_uleft ("/home/dukerama/hydro_catkin/src/serversocket_vision/include/uleft_pixel.txt", std::ofstream::out);
//std::ofstream outfile_uright ("/home/dukerama/hydro_catkin/src/serversocket_vision/include/uright_pixel.txt", std::ofstream::out);

int main ( int argc, char **argv)
{
	// create a text file that will store test results
	ofstream myfile;
	myfile.open("test_result.txt", ios::out|ios::binary);

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

	iLowH = 0;
	iHighH = 255;

	iLowS = 0; 
	iHighS = 255;

	iLowV = 0;
	iHighV = 255;

	remove_small = 5;
	fill_small = 5;

	percent = 0;
        
        Size size(60,60);
        Mat dst_left;
        Mat dst_right;
         
	// create Button	
	buttonLowH	= Rect(0,0,100,100);
	buttonHighH = Rect(100,0,100,100);
	buttonLowS	= Rect(0,100,100,100);
	buttonHighS = Rect(100,100,100,100);
	buttonLowV	= Rect(0,200,100,100);
	buttonHighV	= Rect(100,200,100,100);
	canvas		= Mat3b(buttonLowH.height*3, buttonLowH.width*2, Vec3b(150,150,150));

	canvas(buttonLowH)	= Vec3b(200,200,200);
	canvas(buttonHighH) = Vec3b(200,200,200);
	canvas(buttonLowS)	= Vec3b(200,200,200);
	canvas(buttonHighS) = Vec3b(200,200,200);
	canvas(buttonLowV)	= Vec3b(200,200,200);
	canvas(buttonHighV) = Vec3b(200,200,200);

	//putText(canvas(buttonLowH), "LowH", Point(buttonLowH.width*0.1, buttonLowH.height*0.7), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255,255),2);
	//putText(canvas(buttonHighH), "HighH", Point(buttonLowH.width*0.1, buttonLowH.height*0.7), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255,255), 2);
	//putText(canvas(buttonLowS), "LowS", Point(buttonLowH.width*0.1, buttonLowH.height*0.7), FONT_HERSHEY_PLAIN, 2, Scalar(0,255,0,255), 2);
	//putText(canvas(buttonHighS), "HighS", Point(buttonLowH.width*0.1, buttonLowH.height*0.7), FONT_HERSHEY_PLAIN, 2, Scalar(0,255,0,255), 2);
	//putText(canvas(buttonLowV), "LowV", Point(buttonLowH.width*0.1, buttonLowH.height*0.7), FONT_HERSHEY_PLAIN, 2, Scalar(255,0,0,255), 2);
	//putText(canvas(buttonHighV), "HighV", Point(buttonLowH.width*0.1, buttonLowH.height*0.7), FONT_HERSHEY_PLAIN, 2, Scalar(255,0,0,255), 2);
	setMouseCallback("Control", callBackButton);

	// create Trackbar
	cvCreateTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 255);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);

	cvCreateTrackbar("Remove small objects", "Control", &remove_small, 20); //Value (0 - 20)
	cvCreateTrackbar("Fill in small holes", "Control", &fill_small, 20);

	cvCreateTrackbar("Mask of height(%)", "Control", &percent, 100);
	imshow("Control", canvas);


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
      int socketnum;
      ServerSocket server ( 3100 );
      cout<<"socket created"<<endl;

	  std::clock_t start;
	  std::clock_t total;
	  double duration;

      while ( true )
	  {
		  start = std::clock();

          cout<<"create new socket"<<endl;
		  ServerSocket new_sock;
		  socketnum=server.accept ( new_sock );
          cout<<"accepted socket"<<endl;

		  duration = (std::clock() - start)/(double) CLOCKS_PER_SEC;
		  myfile << "socket creation duration: " << duration <<" sec \n"<<endl;
		  try
		  {
			  total = std::clock();
			  while(ros::ok())
			  {
				  //new code
				  start = std::clock();     
                  camera.takeStereoImages(1);
				  image_left = camera.image_left;    image_right = camera.image_right;
				  
				  imshow("left", image_left);
				  imshow("right", image_right);
				  waitKey(30);
				  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
				  myfile << "image transfer time: " << duration << " sec \n"<<endl;

				  start = std::clock();
				  imageThreshold(image_left, imgThre_left, iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, remove_small, fill_small, percent);
				  imageThreshold(image_right, imgThre_right, iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, remove_small, fill_small, percent);
				  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
				  myfile << "HSV conversion delay: " << duration << " sec \n"<<endl;

				  start = std::clock();
				  Mat src_left=imgThre_left;
				  Mat src_right=imgThre_right;
				  resize(src_left,dst_left,size); 
				  resize(src_right,dst_right,size);  
                   
				  imshow("thre_left", dst_left);
				  imshow("thre_right", dst_right);
				  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
				  myfile << "image resizing time: " << duration << " sec \n"<<endl;

                  //new code//
                  cout<<dst_left.size()<<endl;
                  cout<<dst_right.size()<<endl; 
                  // new code
                  dst_left=(dst_left.reshape(0,1)); //to make it continuous
                  int imgSize=dst_left.total()*dst_left.elemSize();
                 
                  waitKey(30);  
                  std::string data;
                  cout << "sending data\n"<<data;
                  cout << "image size is : \n"<<imgSize<<endl;
				  new_sock >> data;
                
                  int byte;
                  cout<<"the socket number is: "<<server.get()<<endl;

				  start = std::clock();
                  byte=send(socketnum,dst_left.data,imgSize,0);
				  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
				  myfile << "data transfer time: " << duration << " sec \n"<<endl;

                  //cout<<"the data type of dst_left is "<<typeid(dst_left.data).name<<endl;
                  cout<<"the number of byte is:"<<byte<<endl;
                  // std::cout << "running....\n"<<data<<"\nExiting.\n";
                  std::cout << "hello running....\n"<<data;  
                  ros::ServiceClient client=node.serviceClient<irobot_create_2_1::Tank>("tank");
                  irobot_create_2_1::Tank srv;
                  std::cout << "start\n"; 
                                 
                  std::string::size_type sz; 
                  double left=std::stod(data,&sz);
                  double right=std::stod(data.substr(sz)); 
                  
				  start = std::clock();
                  srv.request.left=left;
                  srv.request.right=right;
                  srv.request.clear=1;
                  if(client.call(srv))
				  {
					  ROS_INFO("Service call %d %d",srv.request.left,srv.request.right);
                      if(srv.response.success)
                      {
						  ROS_INFO("Response");
					  }else
                      {
						  ROS_INFO("No Response");
					  }
				  }else
				  {
					  ROS_INFO("Service NOT called");
				  }
				  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
				  myfile << "server response time: " << duration << " sec \n"<<endl;

                  ros::spinOnce();
                  loop_rate.sleep();
                  ++count;
			  }
			  duration = ( std::clock() - total ) / (double) CLOCKS_PER_SEC;
			  myfile << "total run time for single iteration: " << duration << " sec \n"<<endl;
		  }
		  catch ( SocketException& ) {}
	  }
  }

  catch ( SocketException& e )
    {
      std::cout << "Exception was caught:" << e.description() << "\nExiting.\n";
    }

  myfile.close();
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


