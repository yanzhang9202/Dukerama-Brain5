#ifndef ___DETECTOR_LIB_H___
#define ___DETECTOR_LIB_H___

#include "image_lib.cpp"
#include <Eigen/Dense>
#include <fstream>
#include <string>

#include <math.h>

#include "nbv_def.h"

class nbv_detector{
private:
	// Define camera, downsample rate and precision
	pgr_camera camera;
	int img_count;
	int downsample;	// Image downsample rate
	float decimal;	// Circle finder resolution, decimal = 0 --- integer;

	// Load target information
	Eigen::MatrixXi targetInfo;
	void loadTargetInfo();
	int getNumberofLines(std::string filename);

//	// Circle finder
	int max_count;
	int iHighH, iLowH, iHighS, iLowS, iHighV, iLowV, remove_small, fill_small, percent;
	std::vector<cv::Mat> ctr_left, ctr_right;
	std::vector<bool> detected_left, detected_right;
	ros::Rate detect_rate;

	void findCircle();
	bool imageThreshold(const cv::Mat &imgOriginal, cv::Mat &imgThresholded, cv::Mat &result);
	bool checkValidity(const cv::Mat &imgHSV, const cv::Mat &ctr, cv::Mat &result);
	cv::Mat downsampleImg(cv::Mat img);
	void showTargetCtr(cv::Mat img, std::vector<Eigen::Vector2f> target_ctr, string name, bool left);
	void setFlag(std::vector<bool> &flag);
	bool checkFlag(const std::vector<bool> &flag);
	void showFlag(const std::vector<bool> &flag);

	// Show images;
	bool show_original_img;
	bool show_threshold_img;
	bool show_detected_img;

	// Simulation
	bool sim;
	Eigen::Vector2f measure_pixel(Eigen::Vector2f p);
	float flength, bline, wi, fov, alphaq, c;

	// record Pixels
	bool record;

	std::ofstream outfile_left;
	std::ofstream outfile_right;
	std::ofstream outfile_uleft;
	std::ofstream outfile_uright;	

	// Apply correction
	bool corr;
	Eigen::MatrixXf beta;
	Eigen::Vector2f cc_left, cc_right;
	
	void loadCorrBeta();
	void correctPixel(Eigen::Vector2f &left, Eigen::Vector2f &right);

	// Save detection results
	cv::Mat ds_img_left, ds_img_right;

	void saveImages(cv::Mat left, cv::Mat right);

public:
	nbv_detector(ros::NodeHandle &nh, std::string topic_name_left, std::string topic_name_right, std::string server_name, int ds, float dec, bool simu);
	void checkCamera();
	void detectTargets();

	// Simulation
	void simDetect(Eigen::Vector2f r, Eigen::Matrix2f R, Eigen::Vector2f t);

	// record Pixels;
	void closeRecord();

	std::vector<Eigen::Vector2f> target_ctr_left, target_ctr_right;
	int num_targets;
	std::vector<bool> flag_detect;

	void waitForEnter();
};

#endif
