#ifndef ___IMAGE_LIB_CPP___
#define ___IMAGE_LIB_CPP___

#include "image_lib.h"

using namespace std;
using namespace cv;

// Construction function of class
pgr_camera::pgr_camera(ros::NodeHandle &nh, string topic_name_left, string topic_name_right, string server_name):ac(server_name.c_str(), true),loop_rate(10){
	image_transport::ImageTransport it(nh);
	pic_left = it.subscribe(topic_name_left.c_str(),1,boost::bind(&pgr_camera::convert_image, this, _1, &image_left));
	pic_right = it.subscribe(topic_name_right.c_str(),1,boost::bind(&pgr_camera::convert_image, this, _1, &image_right));

 	goal.acquire = true;
//	show_stereo_image = true;
}

// Edit image format
void pgr_camera::convert_image(const sensor_msgs::ImageConstPtr& msg,cv::Mat* image){
  	sensor_msgs::Image imag=*msg;
 	//std::cout << "Image time is: " << imag.header.stamp << std::endl;
  	cv_bridge::CvImagePtr cv_ptr;
  	//printf("Got image\n");
  	ros::spinOnce();
    	try
      	{
        	cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");//sensor_msgs::image_encodings::BGR8);                                                                                                                  
       	}
    	catch (cv_bridge::Exception& e)
      	{
       		ROS_ERROR("cv_bridge exception: %s", e.what());
        	return;
      	}
    	cv::Mat image_out=cv_ptr->image;
    	image_out.copyTo(*image);//left_images.push(image_out);                                                                         
}

// Check camera works normally
void pgr_camera::checkCamera(){
  	bool updated=false;
    	image_left.release(); image_right.release();
  	//ping camera driver and wait for it to start
  	while (ros::ok() && !updated)
  	{
		ac.waitForServer();
		ac.sendGoal(goal);
		if (!image_left.empty() && !image_right.empty())
		{
			imshow("right",image_right);
			imshow("left",image_left);
			cv::waitKey(30);
			updated = true;
		}

		ros::spinOnce();
		loop_rate.sleep();
  	}
	cout << "Check camera works normally!" << endl << endl;		
}

// Take stereo images
void pgr_camera::takeStereoImages(int trial){
	int count = 0;
	while (count < trial){
		//take picture
    		image_left.release(); image_right.release();
    		while (image_left.empty() || image_right.empty()) // No matter what first image is taken, dropped it. Because A queue size of 2 somewhere keeps sending old images.
    		{
    			ac.sendGoal(goal);
    			ac.waitForResult();
    			ros::spinOnce();
			loop_rate.sleep();
    		}
		count = count + 1;
	}
}

// Wair for user enter any key to continue
void pgr_camera::waitForEnter(){
	cout << "Please enter any key to continue! ";
	char enter;
	cin >> enter;
	cout << endl;
}

void pgr_camera::showStereoImages(){
	imshow("right", image_right);
 	imshow("left", image_left);
	cv::waitKey(30);
}

#endif
