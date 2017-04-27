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
	pgr_camera camera;
	int img_count;
	int downsample;	// Image downsample rate
	float decimal;	// Circle finder resolution, decimal = 0 --- integer;
	std::vector<Eigen::Vector3d> target_hsv_lower, target_hsv_upper, target_rgb_lower, target_rgb_upper, target_rgb;
	Eigen::MatrixXf target_corr;

	CircleCtr findCircleHSV(cv::Mat image, Eigen::Vector3d lower, Eigen::Vector3d upper);
//	CircleCtr findCircleRGB(cv::Mat image, Eigen::Vector3d lower, Eigen::Vector3d upper);
	CircleCtr findCircleRGB(cv::Mat image, Eigen::Vector3d rgb, float corr);
	cv::Mat calcCorrMap(cv::Mat image, Eigen::Vector3d rgb);

	void saveImages(cv::Mat left, cv::Mat right);
	cv::Mat downsampleImg(cv::Mat img);
	void loadTargetHSVInfo();
	void loadTargetRGBInfo();
	void showTargetInfo(std::vector<Eigen::Vector3d> target_info);
	int getNumberofLines(std::string filename);
	void checkValidityHSV(cv::Mat imgHSV, cv::Mat ctr, Eigen::Vector3d lower, Eigen::Vector3d upper);
	void checkValidityRGB(cv::Mat imgRGB, cv::Mat ctr, Eigen::Vector3d rgb);
//	void showTargetCtr(cv::Mat img, CircleCtr ctr, string name);
	void showTargetCtr(cv::Mat img, std::vector<Eigen::Vector2f> target_ctr, string name, bool left);

	// Simulation
	bool sim;
	Eigen::Vector2f measure_pixel(Eigen::Vector2f p);
	float flength, bline, wi, fov, alphaq, c;

	void setFlag();

	// record Pixels
	bool record;

	std::ofstream outfile_left;
	std::ofstream outfile_right;

	// Apply correction
	bool corr;
	Eigen::MatrixXf beta;
	Eigen::Vector2f cc_left, cc_right;
	
	void loadCorrBeta();
	void correctPixel(Eigen::Vector2f &left, Eigen::Vector2f &right);

	// Save detection results
	cv::Mat ds_img_left, ds_img_right;

	// Adjust color detector
	bool show_color_map;

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
