#ifndef ___DETECTOR_LIB_CPP___
#define ___DETECTOR_LIB_CPP___

#include "detector_lib.h"

using namespace cv;
using namespace Eigen;
using namespace std;

#define PI 3.1415926

nbv_detector::nbv_detector(ros::NodeHandle &nh, std::string topic_name_left, std::string topic_name_right, std::string server_name, int ds, float dec, bool simu)
	    :camera(nh, topic_name_left, topic_name_right, server_name),
	     outfile_left("/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/Waypoints/left_pixel.txt"),
	     outfile_right("/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/Waypoints/right_pixel.txt"){
	img_count = 600;
	downsample = ds;
	decimal = dec;

	cout << "Initializing circle detector setting..." << endl;
	cout << "Downsampling rate: " << downsample << endl;

	sim = simu;
	if (sim) {
		bline = 1.0;
		wi = 1024.0;
		fov = 70 * (PI / 180);
		alphaq = tan(fov / 2);
		c = 1.0/(2 * alphaq);
		flength = (0.5 * wi) / alphaq;
	}
	if (sim) {
		cout << "flength: " << flength << endl;
	}
	// Initialize number of targets and targets' HSV values
	loadTargetRGBInfo();
	flag_detect.reserve(num_targets);
//	setFlag();
	record = true;

	corr = false;
	if (corr){
		cc_left << 585.91865, 529.58589;
		cc_right << 587.35744, 527.40588;
		cc_left = cc_left / (float)downsample;
		cc_right = cc_right / (float)downsample;
		loadCorrBeta();
	}

	show_color_map = true;
	if (show_color_map){
		namedWindow("Color_map", CV_WINDOW_NORMAL);
	}
}

void nbv_detector::loadCorrBeta(){
	string filename = "/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/Correction/beta.txt";
	ifstream file_beta;
  	file_beta.open(filename.c_str());

  	if (!file_beta.is_open()){
    		cout << "Not enough correction beta information provided!\n";
    	return; 
  	}

	int lines, cols;
	beta.resize(6, 3);
	for (lines = 0; lines < 6; lines++){
		for (cols = 0; cols < 3; cols++){
			file_beta >> beta(lines, cols);	
		}
	}
	
	file_beta.close();

	cout << "Correction Beta: " << endl << beta << endl << endl;
}

void nbv_detector::correctPixel(Vector2f &left, Vector2f &right){
	left = left - cc_left;
	right = right - cc_right;
	MatrixXf corrected, designMat;
	corrected.resize(1,3);
	designMat.resize(1,6);
	float y_bar = (left(1) + right(1))/2.0;
	float xl = left(0);
	float xr = right(0);
	designMat << 1, y_bar, xl - xr, xl + xr, y_bar * (xl - xr), (xl + xr)/(xl - xr);
	corrected = designMat * beta;
	left(0) = corrected(0, 0);
	left(1) = corrected(0, 2);
	right(0) = corrected(0, 1);
	right(1) = corrected(0, 2);
	left = left + cc_left;
	right = right + cc_right;
}

// ************************ Public ************************ //

void nbv_detector::checkCamera(){
	camera.checkCamera();
}

void nbv_detector::detectTargets(){
	target_ctr_left.clear();
	target_ctr_right.clear();
//	setFlag();
	flag_detect.clear();
	if (sim) {
	} else {
		int trial = 3;
		// Take images
		camera.takeStereoImages(trial);
		Mat image_left, image_right;
		image_left = camera.image_left;    image_right = camera.image_right;

		// Test images
//		camera.showStereoImages();

		// Save images
		saveImages(image_left, image_right);

	//	// Detect targets
		CircleCtr ctr_left, ctr_right;
		for (int ind_target = 0; ind_target < num_targets; ind_target++){
	//		Vector3d lower, upper;
			// Use HSV to detect targets
	//		lower = target_hsv_lower[ind_target];	upper = target_hsv_upper[ind_target];
	//		ctr_left = findCircleHSV(image_left, lower, upper);

			// Use RGB to detect targets
	//		lower = target_rgb_lower[ind_target];	upper = target_rgb_upper[ind_target];
	//		ctr_left = findCircleRGB(image_left, lower, upper);
			Vector3d rgb;
			rgb = target_rgb[ind_target];
			ctr_left = findCircleRGB(image_left, rgb, target_corr(ind_target, 0));
			ctr_right = findCircleRGB(image_right, rgb, target_corr(ind_target, 0));
	//		camera.waitForEnter();
			// Store results
			Vector2f temp1, temp2;
			temp1[0] = roundf(ctr_left.ctr.at<float>(0,0) * pow(10, decimal))/pow(10, decimal);
			temp1[1] = roundf(ctr_left.ctr.at<float>(0,1) * pow(10, decimal))/pow(10, decimal);
			temp2[0] = roundf(ctr_right.ctr.at<float>(0,0) * pow(10, decimal))/pow(10, decimal);
			temp2[1] = roundf(ctr_right.ctr.at<float>(0,1) * pow(10, decimal))/pow(10, decimal);

			if ((temp1.norm() < 1) || (temp2.norm() < 1)){
				cout << "Target " << ind_target+1 << " not detected!" << endl << endl;
				flag_detect.push_back(false);
			} else {
				flag_detect.push_back(true);
			}

			// Correction
			if (corr){
//				cout << "Before corrected: " << temp1.transpose() << " --- " << temp2.transpose() << endl;
				correctPixel(temp1, temp2);
//				cout << "After corrected: " << temp1.transpose() << " --- " << temp2.transpose() << endl;	
			}

			target_ctr_left.push_back(temp1);
			target_ctr_right.push_back(temp2);

			if (record){
				outfile_left << temp1[0] << " " << temp1[1] << "\n";
				outfile_right << temp2[0] << " " << temp2[1] << "\n";
			}
		}
		// Show detecting results
		string name = "left";
		showTargetCtr(image_left, target_ctr_left, name, true);
		name = "right";
		showTargetCtr(image_right, target_ctr_right, name, false);
//		saveImages(ds_img_left, ds_img_right);
	}

//	for (int i = 0; i < num_targets; i++){
//		cout << flag_detect.at(i) << " ";
//		if (i == (num_targets-1)){
//			cout << "\n";
//		}
//	}
//	// Press any key to continue
//	camera.waitForEnter();
}

void nbv_detector::simDetect(Vector2f r, Matrix2f R, Vector2f t){
	target_ctr_left.clear();
	target_ctr_right.clear();
	Vector2f p = R.transpose() * (t - r);
	Vector2f pixels = measure_pixel(p);
	Vector2f temp1, temp2;
	temp1[0] = floor(pixels[0]) + 0.5;
	temp1[1] = 0;
	temp2[0] = floor(pixels[1]) + 0.5;
	temp2[1] = 0;
	target_ctr_left.push_back(temp1);
	target_ctr_right.push_back(temp2);
}

void nbv_detector::closeRecord(){
	outfile_left.close();
	outfile_right.close();
}

// ************************* Private *************************** //
Mat nbv_detector::downsampleImg(Mat img){
	cv::Size Size = img.size();
  	int rows = Size.height;
  	int cols = Size.width;
  	Mat new_img;
  	cv::Size newSize(cols/downsample, rows/downsample);
  	resize(img, new_img, newSize);
  	return new_img;
}

CircleCtr nbv_detector::findCircleHSV(Mat image, Vector3d lower, Vector3d upper)
{
  cv::Mat imgHSV;
  cv::cvtColor(image, imgHSV, cv::COLOR_BGR2HSV);
  cv::Mat imgThresholded;

  Scalar lw(lower(0), lower(1), lower(2)), up(upper(0), upper(1), upper(2));

  inRange(imgHSV, lw, up, imgThresholded);
// Detect Orange
//  inRange(imgHSV, cv::Scalar(0, 121, 64), cv::Scalar(18, 241, 184), imgThresholded);
// Detect Green
//  inRange(imgHSV, cv::Scalar(44, 96, 0), cv::Scalar(64, 255, 138), imgThresholded);
// Detect Blue
//  inRange(imgHSV, cv::Scalar(88, 160, 0), cv::Scalar(108, 255, 210), imgThresholded);

//  Some Morphological opening stuff, removing small objects
  erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10,10)));
  dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10,10)));
//  Some Morphological opening stuff, filling small holes
  dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2,2)));
  erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2,2)));

  // Test imgThresholded
  imshow("left", imgThresholded);
  waitKey(30);

  cv::Size size = image.size();
  cv::Mat mask = cv::Mat::zeros(size.height, size.width, CV_8U);
  mask(cv::Rect(0, size.height/2, size.width, size.height-size.height/2)) = 1;
  cv::Mat img_mask;
  imgThresholded.copyTo(img_mask, mask);
//  std::cout << size.height/3 << " " << size.height - size.height/3 << std::endl;

  imgThresholded = downsampleImg(img_mask);

  cv::Mat nonZeros, nZ_converted;
  cv::findNonZero(imgThresholded, nonZeros);
  nonZeros.convertTo(nZ_converted, CV_8UC1);

//  std::cout << "Non zero Locations: " << nonZeros << std::endl;

  cv::Mat row_mean(cv::Mat::zeros(1,2,CV_32SC1));
  cv::reduce(nZ_converted, row_mean, 0, CV_REDUCE_AVG);

//  std::cout << "Detected center: " << row_mean << std::endl; 

  CircleCtr result;

  result.ctr = row_mean;

//  checkValidityHSV(imgHSV, row_mean, lower, upper);

  return result;
}

//CircleCtr nbv_detector::findCircleRGB(Mat image, Vector3d lower, Vector3d upper)
CircleCtr nbv_detector::findCircleRGB(Mat image, Vector3d rgb, float corr)
{
  cv::Mat imgThresholded;

//  Detect target using RGB range
//  Scalar lw(lower(2), lower(1), lower(0)), up(upper(2), upper(1), upper(0));
//  inRange(image, lw, up, imgThresholded);

//  Detect target using correlation map
  Mat corrMap;
  corrMap = calcCorrMap(image, rgb);
  inRange(corrMap, Scalar(corr), Scalar(1.01), imgThresholded);

//  Some Morphological opening stuff, removing small objects
  erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10,10)));
  dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10,10)));
//  Some Morphological opening stuff, filling small holes
  dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2,2)));
  erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2,2)));

  cv::Size size = image.size();
  cv::Mat mask = cv::Mat::zeros(size.height, size.width, CV_8U);
  mask(cv::Rect(0, size.height/3, size.width, size.height-size.height/3)) = 1;
  cv::Mat img_mask;
  imgThresholded.copyTo(img_mask, mask);
//  std::cout << size.height/3 << " " << size.height - size.height/3 << std::endl;

  imgThresholded = downsampleImg(img_mask);

//  Test result
//  imshow("left", imgThresholded);
//  waitKey(30);

  cv::Mat nonZeros, nZ_converted;
  cv::findNonZero(imgThresholded, nonZeros);
  nonZeros.convertTo(nZ_converted, CV_32F);

//  std::cout << "Non zero Locations: " << nonZeros << std::endl;

  cv::Mat row_mean(cv::Mat::zeros(1,2,CV_32F));
  if (!nZ_converted.empty()){
  	cv::reduce(nZ_converted, row_mean, 0, CV_REDUCE_AVG);
	//  std::cout << "Detected center: " << row_mean << std::endl;
  }

  CircleCtr result;

  result.ctr = row_mean;

//  checkValidityRGB(image, row_mean, rgb);

  return result;
}

void nbv_detector::saveImages(Mat left, Mat right){
  img_count++;
  //write image to file
  char filename_left[512];
  sprintf( filename_left, "/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/images/left%d.bmp", img_count );
  char filename_right[512];
  sprintf( filename_right, "/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/images/right%d.bmp", img_count );
  imwrite(filename_left,left);
  imwrite(filename_right,right); 
}

void nbv_detector::loadTargetHSVInfo(){
	string filename_lower = "/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/targetInfo/hsv_target_lower.txt";
	string filename_upper = "/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/targetInfo/hsv_target_upper.txt";
	num_targets = getNumberofLines(filename_lower);
	target_hsv_lower.reserve(num_targets);
	target_hsv_upper.reserve(num_targets);
	target_ctr_left.reserve(num_targets);
	target_ctr_right.reserve(num_targets);

	ifstream file_lower, file_upper;
  	file_lower.open(filename_lower.c_str());
  	file_upper.open(filename_upper.c_str());
    
  	if (!file_lower.is_open() || !file_upper.is_open()){
    		cout << "Not enough target HSV information provided!\n";
    	return; 
  	}

	int lines, cols;
	Vector3d temp;
	for (lines = 0; lines < num_targets; lines++){
		for (cols = 0; cols < 3; cols++){
			file_lower >> temp[cols];	
		}
		target_hsv_lower.push_back(temp);
	}

	for (lines = 0; lines < num_targets; lines++){
		for (cols = 0; cols < 3; cols++){
			file_upper >> temp[cols];		
		}
		target_hsv_upper.push_back(temp);
	}
	
	file_lower.close();
	file_upper.close();

	cout << "Targets' HSV lower bound info: " << endl;
	showTargetInfo(target_hsv_lower);
	cout << endl;
	cout << "Targets' HSV upper bound info: " << endl;
	showTargetInfo(target_hsv_upper);
}

void nbv_detector::loadTargetRGBInfo(){

	if (sim) {
		num_targets = 1;
	} else {
		string filename_lower = "/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/targetInfo/rgb_target_lower.txt";
		string filename_upper = "/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/targetInfo/rgb_target_upper.txt";
		string filename_rgb = "/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/targetInfo/targetRGB.txt";
		num_targets = getNumberofLines(filename_rgb);

		cout << "Number of targets: " << num_targets << endl << endl;

		target_rgb_lower.reserve(num_targets);
		target_rgb_upper.reserve(num_targets);
		target_ctr_left.reserve(num_targets);
		target_ctr_right.reserve(num_targets);

		target_rgb.reserve(num_targets);
		target_corr.resize(num_targets, 1);

		ifstream file_lower, file_upper, file_rgb;
	  	file_lower.open(filename_lower.c_str());
	  	file_upper.open(filename_upper.c_str());
		file_rgb.open(filename_rgb.c_str());
	    
	  	if (!file_lower.is_open() || !file_upper.is_open() || !file_rgb.is_open()){
	    		cout << "Not enough target RGB information provided!\n";
	    	return; 
	  	}

		int lines, cols;
		Vector3d temp;
		for (lines = 0; lines < num_targets; lines++){
			for (cols = 0; cols < 3; cols++){
				file_lower >> temp[cols];	
			}
			target_rgb_lower.push_back(temp);
		}

		for (lines = 0; lines < num_targets; lines++){
			for (cols = 0; cols < 3; cols++){
				file_upper >> temp[cols];		
			}
			target_rgb_upper.push_back(temp);
		}

		for (lines = 0; lines < num_targets; lines++){
			for (cols = 0; cols < 4; cols++){
				if (cols < 3){
					file_rgb >> temp[cols];	
				} else {
					target_rgb.push_back(temp);
					file_rgb >> target_corr(lines, 0);
				}	
			}
		}
	
		file_lower.close();
		file_upper.close();
		file_rgb.close();

//		cout << "Targets' RGB lower bound info: " << endl;
//		showTargetInfo(target_rgb_lower);
//		cout << endl;
//		cout << "Targets' RGB upper bound info: " << endl;
//		showTargetInfo(target_rgb_upper);
//		cout << endl;
		cout << "Targets' RGB bound info: " << endl;
		showTargetInfo(target_rgb);
		cout << "Targets' RGB correlation set: " << target_corr.transpose() << endl << endl;
	}
}

int nbv_detector::getNumberofLines(string filename){
  int number_of_lines = 0;
  ifstream file;
  std::string line;

  file.open(filename.c_str());
    
  if (file.is_open()){
    while(std::getline(file, line))
    {
      ++number_of_lines;
    }
    return number_of_lines;
  }
  else
  { return 0; }
}

void nbv_detector::showTargetInfo(vector<Vector3d> target_info){
	Vector3d temp;
	for (std::vector<Eigen::Vector3d>::iterator it = target_info.begin(); it != target_info.end(); ++it){
		temp = *it;
		cout << temp.transpose() << endl;
	}
}

void nbv_detector::checkValidityHSV(cv::Mat imgHSV, cv::Mat ctr, Eigen::Vector3d lower, Eigen::Vector3d upper){
//	cv::Mat img = downsampleImg(imgHSV);
	int x_ctr = (int)ctr.at<uchar>(0,0), y_ctr = (int)ctr.at<uchar>(0, 1);
	cout << "Detected center coordinates: " << x_ctr <<  " " << y_ctr << endl << endl;
	Vec3b pixel = imgHSV.at<Vec3b>(downsample*x_ctr, downsample*y_ctr);
	cout << "HSV at center pixel: " << " H - " << (int)pixel[0] << " S - " << (int)pixel[1] << " V - " << (int)pixel[2] << endl << endl;
}

void nbv_detector::checkValidityRGB(cv::Mat imgRGB, cv::Mat ctr, Eigen::Vector3d rgb){
//	cv::Mat img = downsampleImg(imgRGB);
	int x_ctr = (int)ctr.at<uchar>(0,0), y_ctr = (int)ctr.at<uchar>(0, 1);
	cout << "Detected center coordinates: " << x_ctr <<  " " << y_ctr << endl << endl;
	Vec3b pixel = imgRGB.at<Vec3b>(downsample * x_ctr, downsample * y_ctr);
	cout << "RGB at center pixel: " << " R - " << (int)pixel[2] << " G - " << (int)pixel[1] << " B - " << (int)pixel[0] << endl << endl;
}

void nbv_detector::showTargetCtr(cv::Mat img, std::vector<Eigen::Vector2f> target_ctr, string name, bool left){
	cv::Mat img_ds = downsampleImg(img);
//       	cv::Point pt((int)ctr.ctr.at<float>(0,0), (int)ctr.ctr.at<float>(0,1));    
//	circle( img_ds, pt, 1, cv::Scalar(0, 0, 255), -1, 8, 0 );
	for (int i = 0; i < num_targets; i++){
		if (target_ctr[i][0] > 0 && target_ctr[i][1] > 0) {
			cv::Point pt((int)(target_ctr[i][0]), (int)(target_ctr[i][1]));
			circle( img_ds, pt, 1, cv::Scalar(0, 0, 255), -1, 8, 0 );
		} else {
			cout << "Target " << i+1 << " not shown!" << endl << endl;
		}
	}
	imshow(name.c_str(),img_ds);
        cv::waitKey(30);
	if (left){
		ds_img_left = img_ds;	
	} else {
		ds_img_right = img_ds;
	}
}

Mat nbv_detector::calcCorrMap(Mat image, Vector3d rgb){
	Mat channel[3], temp, temp1, temp2, temp3;
	split(image, channel);
	Mat fB, fG, fR, fnorm;
	channel[0].convertTo(fB, CV_32F);
	channel[1].convertTo(fG, CV_32F);
	channel[2].convertTo(fR, CV_32F);
	magnitude(fB, fG, temp);
	magnitude(temp, fR, fnorm);

	Size imgSize = image.size();
	Mat tB(imgSize, CV_32F, rgb[2]);
	Mat tG(imgSize, CV_32F, rgb[1]);
	Mat tR(imgSize, CV_32F, rgb[0]);

	multiply(fB, tB, temp1); multiply(fG, tG, temp2); multiply(fR, tR, temp3);
	temp = temp1 + temp2 + temp3;
	divide(temp, fnorm, temp1, 1/rgb.norm());
	
	// Test the correlation Map
	if (show_color_map){
	imshow("Color_map", temp1);
	waitKey(30);
	}
//	Mat dst;
//	normalize(temp1, dst, 0, 1, NORM_MINMAX);

	return temp1;
}

//******************* Simulation *******************//
Eigen::Vector2f nbv_detector::measure_pixel(Eigen::Vector2f p){
	Vector2f pixels;
	pixels[0] = flength * (p[0] + 0.5) / p[1];
	pixels[1] = flength * (p[0] - 0.5) / p[1];
	return pixels;
}

void nbv_detector::setFlag(){
	for (int i = 0; i < num_targets; i++){
		flag_detect.push_back(false);
	}
}

void nbv_detector::waitForEnter(){
	char enter;
	cin >> enter;
	cin.clear();
	cin.sync();
}

#endif
