#ifndef ___TARGET_H___
#define ___TARGET_H___

#include "nbv_def.h"
#include <Eigen/Dense>

class target{
private:
  	// *********** Kalman Filter update ************ //
//  	Vector3f z_old_pred;
//  	Vector3f z_new_pred;
//  	Matrix3f U_old_pred;
//  	Matrix3f U_new_pred;
//  	Matrix3f K;	// Kalman filter gain
//  	Matrix3f H;
//  	Matrix3f W;
//  	Matrix3f phi;
	Matrix<float, 9, 1> z_old_pred, z_new_pred;
	Matrix<float, 9, 9> U_old_pred, U_new_pred, W, phi;
	Matrix<float, 9, 3> K;
	Matrix<float, 3, 9> H;
public:
	target();

	void KalmanFilter(int index);

  	std::vector<Vector3f> meas_history;
  	std::vector<Matrix3f> measCov_history;
  	std::vector<Vector3f> ftr_meas_history;
  	std::vector<Matrix3f> ftr_measCov_history;
	std::vector<Vector3f> ftr_pk_history;
	std::vector<Matrix3f> Uk_pred_history;
};

#endif
