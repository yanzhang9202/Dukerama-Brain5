#ifndef ___LOCALIZER_LIB_H___
#define ___LOCALIZER_LIB_H___

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <string.h>

#include "target.cpp"
#include "nbv_def.h"

class nbv_localizer{
private:
	void getCameraCalib();
	void getCameraUncertainty();
	void vrpnCallback(const geometry_msgs::TransformStamped msg);
	Eigen::Vector3f triangulation(float xl, float xr, float y);
	Matrix3f calc_sigma(float xl, float xr, float y);
	Matrix3f calc_J(float xl, float xr, float y);

	float downsample;
	float flength, bline;
	Eigen::Vector2f cc_left, cc_right;
	Eigen::Matrix4f Hc2m_left, Hc2m_right;
	Eigen::Matrix3f Rot_c2h, Rot_h2w, Rot_c2w, Rot_w2c;
	Eigen::Vector4f q_h;
	Eigen::Vector3f ocam_hand, ohand_world, ocam_world;
	std::vector<Eigen::Vector2f> ctr_left, ctr_right;
	ros::Subscriber vrpn_sub;
	int num_targets;
	int count_meas;

	float xl, xr, y;
	Eigen::Matrix3f Q;

	// Simulation
	bool sim;
	float wi, fov, alphaq, c;

	Eigen::Vector3f simtriangulation(float xl, float xr, float y, Eigen::Vector2f r, Eigen::Matrix2f R);
	Eigen::Matrix3f simcalc_sigma(float xl, float xr, float y, Eigen::Vector2f r, Eigen::Matrix2f R);
	Eigen::Matrix2f matrix2d(int index1, int index2, Eigen::Matrix3f mat);

	// Record measurements
	std::ofstream outfile_meas;
	std::ofstream outfile_measCov;
	std::ofstream outfile_ftrm;
	std::ofstream outfile_ftrCov;

	void writeMatrix(std::ofstream& out, Eigen::MatrixXf matrix);
public:
	nbv_localizer(ros::NodeHandle &nh, int ds, int num_t, bool simu);

	void test_vrpn();
	void calcMeasurements(std::vector<Eigen::Vector2f> target_ctr_left, std::vector<Eigen::Vector2f> target_ctr_right, std::vector<bool> flag);
	void showMeasurements(int index);
	void showFtrMeasurements(int index);

	std::vector<target *> targets;

	// Simulation
	void simMeasurements(std::vector<Eigen::Vector2f> target_ctr_left, std::vector<Eigen::Vector2f> target_ctr_right, Eigen::Vector2f r, Eigen::Matrix2f R);
};

#endif
