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
	     outfile_right("/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/Waypoints/right_pixel.txt"),
	     outfile_uleft("/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/Waypoints/uleft_pixel.txt"),
	     outfile_uright("/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/Waypoints/uright_pixel.txt"),
	     detect_rate(5){
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
	loadTargetInfo();
	flag_detect.reserve(num_targets);
	detected_left.reserve(num_targets);
	detected_right.reserve(num_targets);
	ctr_left.reserve(num_targets);
	ctr_right.reserve(num_targets);
	for (int i = 0; i < num_targets; i++){
		ctr_left.push_back(cv::Mat::zeros(1,2,CV_32F));
		ctr_right.push_back(cv::Mat::zeros(1,2,CV_32F));
	}

	record = true;

	corr = true;
	if (corr){
		cc_left << 585.91865, 529.58589;
		cc_right << 587.35744, 527.40588;
		cc_left = cc_left / (float)downsample;
		cc_right = cc_right / (float)downsample;
		loadCorrBeta();
	}

	show_original_img = true;
	show_threshold_img = false;
	show_detected_img = true;
	if(show_original_img){
  		namedWindow("right",CV_WINDOW_NORMAL);
 		namedWindow("left",CV_WINDOW_NORMAL);		
	}
	if(show_threshold_img){
		namedWindow("thre_right",CV_WINDOW_NORMAL);
  		namedWindow("thre_left",CV_WINDOW_NORMAL);
	}
	if(show_detected_img){
		namedWindow("detected_right",CV_WINDOW_NORMAL);
		namedWindow("detected_left",CV_WINDOW_NORMAL);
	}

	max_count = 5;
}

// ************************* Private *************************** //


void nbv_detector::loadTargetInfo(){
	string filename = "/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/targetInfo/targetHSV.txt";
	ifstream file;
  	file.open(filename.c_str());

  	if (!file.is_open()){
    		cout << "Not enough target HSV information provided!\n";
    	return; 
  	}

	num_targets = getNumberofLines(filename);

	targetInfo.resize(num_targets, 9);
	for (int lines = 0; lines < num_targets; lines++){
		for (int cols = 0; cols < 9; cols++){
			file >> targetInfo(lines, cols);	
		}
	}
	
	file.close();

	cout << "Target HSV Info: " << endl << targetInfo << endl << endl;
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

int nbv_detector::getNumberofLines(string filename){
	int number_of_lines = 0;
	ifstream file;
	std::string line;

	file.open(filename.c_str());

	if (file.is_open()){
		while(std::getline(file, line)){
			++number_of_lines;
		}
		return number_of_lines;
	}
	else { return 0; }
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

// Find circle and return results;
void nbv_detector::findCircle(){
	// Initialize
	target_ctr_left.clear();
	target_ctr_right.clear();
	flag_detect.clear();
	setFlag(detected_left);
	setFlag(detected_right);
//	cout << "detected_left: " << detected_left.size() << endl;
//	showFlag(detected_left);

	// Start detection
	int detected_count = 0;
	cv::Mat image_left, image_right;
	cv::Mat imgThre_left, imgThre_right;
	while(ros::ok() && (!checkFlag(detected_left) || !checkFlag(detected_right)) && detected_count < max_count){
		// Take images
		int trial = 3;
		camera.takeStereoImages(trial);
		image_left = camera.image_left;    image_right = camera.image_right;
		if (show_original_img){
			imshow("left", image_left);
			imshow("right", image_right);
			waitKey(100);	
		}
		if (!image_left.empty() && !image_right.empty()){
			for (int i = 0; i < num_targets; i++){
				if (!detected_left[i] || !detected_right[i]){
					iLowH = targetInfo(i, 0); iHighH = targetInfo(i, 1);
					iLowS = targetInfo(i, 2); iHighS = targetInfo(i, 3);
					iLowV = targetInfo(i, 4); iHighV = targetInfo(i, 5);
					remove_small = targetInfo(i, 6);
					fill_small = targetInfo(i, 7);
					percent = targetInfo(i, 8);
					if (!detected_left[i]){
						detected_left[i] = imageThreshold(image_left, imgThre_left, ctr_left[i]);
					}
					if (!detected_right[i]){
						detected_right[i] = imageThreshold(image_right, imgThre_right, ctr_right[i]);
					}
					// Show threshold image
					if (show_threshold_img){
//						cout << detected_count+1 << " th detection " << " Target " << i+1 << endl;
						imshow("thre_left", imgThre_left);
						imshow("thre_right", imgThre_right);
						waitKey(1000);	
					}
				}
			}
			detected_count++;
		}
		detect_rate.sleep();
	}
//	cout << "detected_count: " << detected_count << endl << endl;

	for (int i = 0; i < num_targets; i++){
		if (detected_left[i] && detected_right[i]){
			flag_detect.push_back(true);			
		} else {
			cout << "Target " << i+1 << " not detected!" << endl << endl;
			flag_detect.push_back(false);
		}
	}

//	cout << "flag_detect: " << endl;
//	showFlag(flag_detect);

	// Record (and correct) pixel coordinates with designated decimals in downsample resolution
	for (int i = 0; i < num_targets; i++){
		Vector2f temp1, temp2;
		temp1[0] = roundf(ctr_left[i].at<float>(0,0) / (float)downsample * pow(10, decimal))/pow(10, decimal);
		temp1[1] = roundf(ctr_left[i].at<float>(0,1) / (float)downsample * pow(10, decimal))/pow(10, decimal);
		temp2[0] = roundf(ctr_right[i].at<float>(0,0) / (float)downsample * pow(10, decimal))/pow(10, decimal);
		temp2[1] = roundf(ctr_right[i].at<float>(0,1) / (float)downsample * pow(10, decimal))/pow(10, decimal);

		if (record){
			outfile_uleft << temp1[0] << " " << temp1[1] << "\n";
			outfile_uright << temp2[0] << " " << temp2[1] << "\n";
		}

//		cout << "Before corrected: " << temp1.transpose() << " --- " << temp2.transpose() << endl;
		if (corr){
			correctPixel(temp1, temp2);
//			cout << "After corrected: " << temp1.transpose() << " --- " << temp2.transpose() << endl;	
		}

		target_ctr_left.push_back(temp1);
		target_ctr_right.push_back(temp2);

		if (record){
			outfile_left << temp1[0] << " " << temp1[1] << "\n";
			outfile_right << temp2[0] << " " << temp2[1] << "\n";
		}
	}

	// Show detected images
	if (show_detected_img){ // Show detecting results
		string name = "detected_left";
		showTargetCtr(image_left, target_ctr_left, name, true);
		name = "detected_right";
		showTargetCtr(image_right, target_ctr_right, name, false);
//		saveImages(ds_img_left, ds_img_right);
	}
}

bool nbv_detector::imageThreshold(const Mat &imgOriginal, Mat &imgThresholded, Mat &result){
	Mat imgHSV;

	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
	      
	//morphological opening (remove small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(remove_small, remove_small)) );
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(remove_small, remove_small)) ); 

	//morphological closing (fill small holes in the foreground)
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(fill_small, fill_small)) ); 
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(fill_small, fill_small)) );

	// Mask the non-ROI
	cv::Size size = imgOriginal.size();
	cv::Mat mask = cv::Mat::zeros(size.height, size.width, CV_8U);
	mask(cv::Rect(0, percent*size.height/100, size.width, size.height-percent*size.height/100)) = 1;
	cv::Mat img_mask;
	imgThresholded.copyTo(img_mask, mask);
	imgThresholded = img_mask;

//	cout << "Here!" << endl;
//	imshow("thre_left", imgThresholded);
//	cv::waitKey(0);
//	waitForEnter();
	
	// Calculate and verify detected center
  	cv::Mat nonZeros, nZ_converted;
  	cv::findNonZero(imgThresholded, nonZeros);
  	nonZeros.convertTo(nZ_converted, CV_32F);
//	cout << "nZ_converted: " << nZ_converted << endl;
	cv::Mat row_mean(cv::Mat::zeros(1,2,CV_32F));
	if (!nZ_converted.empty()){
		cv::reduce(nZ_converted, row_mean, 0, CV_REDUCE_AVG);
//		cout << "Detected center: " << row_mean << endl;
		return checkValidity(imgHSV, row_mean, result);
	} else {
		result = row_mean;
		return false;	// Nothing detected
	}
}

bool nbv_detector::checkValidity(const Mat &imgHSV, const Mat &ctr, Mat &result){
	int x_ctr = (int)ctr.at<float>(0,0), y_ctr = (int)ctr.at<float>(0,1);
//	cout << "Detected center coordinates: " << x_ctr <<  " " << y_ctr << endl << endl;
	Vec3b pixel = imgHSV.at<Vec3b>(y_ctr, x_ctr);	// Note the ordering of x and y coordinates, y - row of matrix, x - col of matrix
//	cout << "HSV at center pixel: " << " H - " << (int)pixel[0] << " S - " << (int)pixel[1] << " V - " << (int)pixel[2] << endl << endl;
	if ((int)pixel[0] >= iLowH && (int)pixel[0] <= iHighH){
		result = ctr;
		return true;
	} else {
		cout << "False detection!" << endl;
		result = cv::Mat::zeros(1,2,CV_32F);
		return false;
	}
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

Mat nbv_detector::downsampleImg(Mat img){
	cv::Size Size = img.size();
  	int rows = Size.height;
  	int cols = Size.width;
  	Mat new_img;
  	cv::Size newSize(cols/downsample, rows/downsample);
  	resize(img, new_img, newSize);
  	return new_img;
}

void nbv_detector::showTargetCtr(cv::Mat img, std::vector<Eigen::Vector2f> target_ctr, string name, bool left){
	cv::Mat img_ds = downsampleImg(img);
	for (int i = 0; i < num_targets; i++){
		if (target_ctr[i][0] > 0 && target_ctr[i][1] > 0) {
			cv::Point pt((int)(target_ctr[i][0]), (int)(target_ctr[i][1]));
			circle( img_ds, pt, 1, cv::Scalar(0, 0, 255), -1, 8, 0 );
		} else {
			cout << "Target " << i+1 << " not shown!" << endl;
		}
	}
	imshow(name.c_str(),img_ds);
        cv::waitKey(100);
	if (left){
		ds_img_left = img_ds;	
	} else {
		ds_img_right = img_ds;
	}
}

// ************************ Public ************************ //

void nbv_detector::checkCamera(){
	camera.checkCamera();
}

void nbv_detector::detectTargets(){
	// Detect circle
	findCircle();
}

//******************* Simulation *******************//
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

Eigen::Vector2f nbv_detector::measure_pixel(Eigen::Vector2f p){
	Vector2f pixels;
	pixels[0] = flength * (p[0] + 0.5) / p[1];
	pixels[1] = flength * (p[0] - 0.5) / p[1];
	return pixels;
}

void nbv_detector::closeRecord(){
	outfile_left.close();
	outfile_right.close();
}

void nbv_detector::setFlag(std::vector<bool> &flag){
	if (flag.size() == 0){
		for (int i = 0; i < num_targets; i++){
			flag.push_back(false);
		}
	} else {
		flag.clear();
		for (int i = 0; i < num_targets; i++){
			flag.push_back(false);
		}
	}
}

bool nbv_detector::checkFlag(const std::vector<bool> &flag){
	if (flag.size() < 1 || flag.size() > num_targets){
		cout << "Error! The flag vector is of size " << flag.size() << " !" << endl;
		exit(EXIT_FAILURE);
		return false;
	} else {
		bool check = true;
		for (int i = 0; i < num_targets; i++){
			check = check && flag[i];
		}
		return check;
	}
}

void nbv_detector::showFlag(const std::vector<bool> &flag){
	if (flag.size() < 1 || flag.size() > num_targets){
		cout << "Error! The flag vector is of size " << flag.size() << " !" << endl;
		exit(EXIT_FAILURE);
	} else {
		for (int i = 0; i < num_targets; i++){
			cout << flag[i] << " ";
		}
		cout << endl << endl;
	}
}

void nbv_detector::waitForEnter(){
	cout << "Please enter any key to continue:" << endl;
	char enter;
	cin >> enter;
	cin.clear();
	cin.sync();
}

#endif
