//
//  stereo_image_subscriber.cpp
//  
//
//  Created by Alex Zhu on 9/11/13.
//
//

#ifndef ___stereo_image_subscriber_cpp
#define ___stereo_image_subscriber_cpp

#include "stereo_image_subscriber.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

void stereo_image_subscriber::convert_image(const sensor_msgs::ImageConstPtr& msg,bool left){
  cv_bridge::CvImagePtr cv_ptr;
  //printf("Got image\n");
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");//sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //ROS_DEBUG("Hiii");
    cv::Mat image_out=cv_ptr->image;

    //cv::Mat image_temp,image_out;
    //ROS_DEBUG("Downsampling");
    //cv::pyrDown(image,image_out,cv::Size((image.cols)/2,(image.rows)/2));
    //cv::pyrDown(image_temp,image_out,cv::Size((image_temp.cols)/2,(image_temp.rows)/2));

    //cv::namedWindow("test", CV_WINDOW_NORMAL);
    //cv::imshow( "test", image_out );
    //int c=cv::waitKey(2000);
        
    if (left) {
      image_out.copyTo(left_image);//left_images.push(image_out);
      left_updated=true;
      /*sensor_msgs::Image image=*msg;
	std::cout << "Image time is: " << image.header.stamp << std::endl;*/
    }
    else {
      image_out.copyTo(right_image);//right_images.push(image_out);
      right_updated=true;
    }
}

stereo_image_subscriber::stereo_image_subscriber(ros::NodeHandle nh):it(nh),ac("/take_pic",true)
{
  pic_left=it.subscribe("/stereo/left/image_rect_color",1,boost::bind(&stereo_image_subscriber::convert_image,this,_1,true));
  pic_right=it.subscribe("/stereo/right/image_rect_color",1,boost::bind(&stereo_image_subscriber::convert_image,this,_1,false));
  left_updated=false;
  right_updated=false;
  takepic.acquire=true;
  ac.waitForServer();
}

void stereo_image_subscriber::take_photos(){
  ac.sendGoal(takepic);
  ac.waitForResult(ros::Duration(0.0));
}

#endif
