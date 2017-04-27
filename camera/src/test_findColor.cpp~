#include <ros/ros.h>
#include <iostream>
#include <string>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include "image_lib.cpp"

using namespace cv;
using namespace std;

void imageThreshold(const Mat &imgOriginal, Mat &imgThresholded, int iLowH, int iHighH, int iLowS, 
		    int iHighS, int iLowV, int iHighV, int remove_small, int fill_small, int percent);

int main(int argc, char **argv){
	ros::init(argc, argv, "test_findColor");
	ros::NodeHandle n;

	//// Create two windows to show image
	namedWindow("right",CV_WINDOW_NORMAL);
	namedWindow("left",CV_WINDOW_NORMAL);

	namedWindow("thre_right",CV_WINDOW_NORMAL);
	namedWindow("thre_left",CV_WINDOW_NORMAL);	

	//// Declare robot name, server name and image topics' names
	string robotname, server_name, topic_name_left, topic_name_right;
	n.getParam("/robot/name", robotname); 
	server_name = robotname+"/take_pic";
	topic_name_left = robotname+"/left/image_rect_color";		// Take raw images or rectified images
	topic_name_right = robotname+"/right/image_rect_color";

	pgr_camera camera(n, topic_name_left, topic_name_right, server_name);

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

	camera.checkCamera();

	Mat image_left, image_right;
	Mat imgThre_left, imgThre_right;

	while(ros::ok()){
		// Take images
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
                cout<<dst_left.size()<<endl;
                cout<<dst_right.size()<<endl;
		waitKey(30);
	
//		loop_rate.sleep();
	}
}

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

