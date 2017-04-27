//
//  stereo_localisation.cpp
//  
//
//  Created by Alex Zhu on 9/11/13.
//
//

#ifndef ___stereo_localisation_cpp
#define ___stereo_localisation_cpp

#include "stereo_localisation.h"

#define PRINCIPAL_X_L 649.9451//629.09825
#define PRINCIPAL_Y_L 514.0357//510.5
#define PRINCIPAL_X_R 621.4512//626.54556
#define PRINCIPAL_Y_R 514.0357//513.79758


// Threshold Mat image to only show colors within the given color range
void stereo_localisation::get_target_coords(cv::Mat img,bool isLeft, int color, int hue1,int hue2, int param1, int param2,int sat, int val, int min_radius, int max_radius)
{
  int hues[8] = {/*100,110*/6,10,60,80,100,110,130,160};//105 110

  //  int hue1=0;
  //int hue2=0;

  cv::Mat imgHSV, imgThreshed, imgOut;
  
  /*if (color==3){
    coords.push_back(-1);
    coords.push_back(-1);
    return;
    }*/

  
    if (0<=color && color <=3){
      hue1=hues[(color*2)];
      hue2=hues[(color*2+1)];

    } else if (color!=-1){
      std::cout << "Error: Incorrect color value entered" << std::endl;
        return;
    }

    cv::cvtColor(img, imgHSV, CV_BGR2HSV);

    cv::inRange(imgHSV, cv::Scalar (hue1,sat,val), cv::Scalar(hue2,255,255), imgThreshed);
    
    cv::GaussianBlur(imgThreshed,imgThreshed,cv::Size(5,5),2,2);
    //cv::namedWindow("Left", CV_WINDOW_NORMAL);
    //cv::namedWindow("Right", CV_WINDOW_NORMAL);
    std::vector<cv::Vec3f> circles;
    for (int j=0;j<20;j++){
      circles.clear();
      cv::HoughCircles(imgThreshed,circles,CV_HOUGH_GRADIENT,1,imgThreshed.cols/1.1,param1,param2,min_radius,max_radius);
      if (circles.size()==1){
	if (isLeft){
	  coords.push_back(circles[0][0]); //-PRINCIPAL_X_L
	  //printf("image xl is: %f, resolved xl is: %f\n",circles[0][0],circles[0][0]-PRINCIPAL_X_L);
	  coords.push_back(circles[0][1]); //PRINCIPAL_Y_L-
	  radius=cvRound(circles[0][2]);
	} else {
	  coords.push_back(circles[0][0]); //-PRINCIPAL_X_R
	  //printf("image xr is:%f, resolved xr is: %f\n",circles[0][0], circles[0][0]-PRINCIPAL_X_R);
	  //coords.push_back(PRINCIPAL_Y_R-circles[0][1]);
	}
      }
        
      if (circles.size()==1){
	break;
        }
    }

    if (circles.size()!=1){
      coords.push_back(TARGET_ERROR);
      if (isLeft){
	coords.push_back(TARGET_ERROR);
      }
    }

#ifdef SHOWIMAGE
    //cv::cvtColor(imgThreshed,imgOut,CV_GRAY2RGB);
    cv::cvtColor(img,imgOut,CV_HSV2RGB);
    
    for (int i=0;i<circles.size();i++){
      cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      //std::cout << "Radius= " << radius << std::endl; //<< " Point x= " << circles[i][0] << " Point y= " << circles[i][1] << endl;
      circle(imgOut, center, 3 , cv::Scalar(0,255,0), 3,8,0);
      circle(imgOut,center,radius,cv::Scalar(0,0,255),3,8,0);
    }

    if (isLeft){
      std::cout << "Showing left" << std::endl;
      cv::imshow("Left",imgOut);
      cv::waitKey(10);
      char filename_left[512];
      sprintf( filename_left, "/home/dukerama/catkin_ws/src/single_localization/include/detector/left.bmp");
      imwrite(filename_left,imgOut);
    } else {
      cv::imshow("Right",imgOut);
      cv::waitKey(10);
      char filename_right[512];
      sprintf( filename_right, "/home/dukerama/catkin_ws/src/single_localizatoin/include/detector/right.bmp");
      imwrite(filename_right,imgOut);
      }
#endif
    return;
}

cv::Mat stereo_localisation::localise_targets(cv::Mat image_left, cv::Mat image_right){
    	coords.clear();
	for (int i=0;i<NUM_TARGETS;i++){
	    get_target_coords(image_left,true,i,100,110,350,9,50,40,0,100);
	    if (coords[i*3]==TARGET_ERROR) {
	      coords.push_back(TARGET_ERROR);
	      continue;
	    }
	    get_target_coords(image_right,false,i,100,110,350,9,50,40,0,100);
	    if (coords[i*3+2]==TARGET_ERROR){
	      coords[i*3]=TARGET_ERROR;
	      coords[i*3+1]=TARGET_ERROR;
	    }
	}
//  for (int i=0;i<NUM_TARGETS;i++){
//    get_target_coords(image_left,true,i,100,110,350,9,50,40,0,100);
//    if (coords[i*3]==TARGET_ERROR) {
//      coords.push_back(TARGET_ERROR);
//      continue;
//    }
//    
//    double leeway=10;
//    double ycoord=std::max(PRINCIPAL_Y_L-coords[i*3+1]-radius-leeway/2,0.0);
//    int height=std::min(radius*2+leeway,image_right.rows-ycoord);
//    //printf("y is: %f, ycord is: %f, radius is: %d\n",coords[i*4+1],ycoord,radius);
//    get_target_coords(image_right(cv::Rect(0,ycoord,image_right.cols,height)),false,i,100,110,400,10,50,30,radius-5,radius+5);
//    if (coords[i*3+2]==TARGET_ERROR){
//      coords[i*3]=TARGET_ERROR;
//      coords[i*3+1]=TARGET_ERROR;
//      //printf("Target %d not foundd\n",i);
//    }
//    //printf("xl is:%f, xr is:%f\n",coords[i*4],coords[i*4+2]);
//  }

  	cv::Mat imgOut;
  	cv::cvtColor(image_left,imgOut,CV_HSV2RGB);
	  for (int i=0;i<NUM_TARGETS;i++){
	    cv::Point center(cvRound(coords[i*3])+PRINCIPAL_X_L, PRINCIPAL_Y_L-cvRound(coords[i*3+1]));
	    //std::cout << "Radius= " << radius << std::endl; //<< " Point x= " << circles[i][0] << " Point y= " << circles[i][1] << endl;
	    circle(imgOut, center, 3 , cv::Scalar(0,255,0), 3,8,0);
	  }

	  return imgOut;
}

stereo_localisation::stereo_localisation(int n){
  N=n;
  coords.reserve(NUM_TARGETS*3);
}
        
#endif
