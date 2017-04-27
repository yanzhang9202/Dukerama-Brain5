#ifndef _TARGET_CPP_
#define _TARGET_CPP_

#include "target.h"

using namespace Eigen;

target::target(){
	meas_history.reserve(NUM_MEAS);
	measCov_history.reserve(NUM_MEAS);
	ftr_meas_history.reserve(NUM_MEAS);
	ftr_measCov_history.reserve(NUM_MEAS);
	ftr_pk_history.reserve(NUM_MEAS);

	// Measurement matrix
  	H = Matrix<float,3,9>::Zero();
  	H(0,0) = 1;
  	H(1,1) = 1;
  	H(2,2) = 1;

	// State transition matrix
 	phi = Matrix<float,9,9>::Zero();
  	phi(0,0) = 1;
  	phi(1,1) = 1;
  	phi(2,2) = 1;

	// Initialize Kalman Filter gain
  	K = Matrix<float,9,3>::Zero();

	// Process Noise model 1 --- Singer Correlated Process Model
//  	for (int i=0;i<3;i++){
//    		for (int j=0;j<3;j++){
//      			W(i,j) = pow(kf_delta,5)/20; 
//      			W(i,3+j) = pow(kf_delta,4)/8;
//      			W(3+i,j) = pow(kf_delta,4)/8;
//      			W(3+i,3+j) = pow(kf_delta,3)/3;
//      			W(6+i,j) = pow(kf_delta,3)/6;
//      			W(i,6+j) = pow(kf_delta,3)/6;
//      			W(6+i,3+j) = kf_delta*kf_delta/2;
//      			W(3+i,6+j) = kf_delta*kf_delta/2;
//      			W(6+i,6+j) = kf_delta;
//    		}
//    	}
	
	// Process Noise model 2 --- Diagonal model
	W = Matrix<float, 9, 9>::Identity() * kf_delta;

//	cout << "Process noise: " << endl;
//	cout << W << endl;

	// Initialize state vector
	z_old_pred = Matrix<float, 9, 1>::Zero();
	z_new_pred = Matrix<float, 9, 1>::Zero();

	// Initialize covariance matrix
	U_old_pred = Matrix<float, 9, 9>::Zero();
	U_new_pred = Matrix<float, 9, 9>::Zero();
}

void target::KalmanFilter(int index){
  	Matrix3f sigma_meas = measCov_history[index];
  	Vector3f x_meas = meas_history[index];

//	cout << "index: " << index << endl;
//	cout << "sigma_meas: " << endl << sigma_meas << endl;
//	cout <<  "x_meas: " << x_meas.transpose() << endl;
 
  	// Next prediction generated based on previous prediction and measurement
  	if (index > 0){  // Do Kalman Filtering
    		z_old_pred = phi * z_new_pred; // Predite State
    		U_old_pred = phi * U_new_pred * phi.transpose() + W; // Prediction Noise
    		K = U_old_pred * H.transpose() * ((H * U_old_pred * H.transpose() + sigma_meas).inverse()); // Optimal Kalman Gain
    		z_new_pred = z_old_pred + K * (x_meas - H * z_old_pred); // Updated state estimate
    		U_new_pred = U_old_pred - K * H * U_old_pred; // Updated State Covariance
	} else {      // First prediction generated based on uncertainty matrix and framerate
    		for (int i = 0; i < 3; i++){
			z_new_pred.setZero();
			U_new_pred = 100.0 * Matrix<float, 9, 9>::Identity();
    			z_old_pred = phi * z_new_pred; // Predite State
    			U_old_pred = phi * U_new_pred * phi.transpose() + W; // Prediction Noise
    			K = U_old_pred * H.transpose() * ((H * U_old_pred * H.transpose() + sigma_meas).inverse()); // Optimal Kalman Gain
    			z_new_pred = z_old_pred + K * (x_meas - H * z_old_pred); // Updated state estimate
    			U_new_pred = U_old_pred - K * H * U_old_pred; // Updated State Covariance

//      			for (int j = 0; j < 3; j++){
//				U_new_pred(i,j) = sigma_meas(i,j);
//      			}
//    		}

//    		for (int i = 0; i < 3; i++){
//      			z_new_pred(i) = x_meas(i);
//    		}
//    		U_old_pred = U_new_pred;
//    		z_old_pred = z_new_pred;
		}
  	}
  	ftr_meas_history.push_back(H * z_new_pred);
	ftr_measCov_history.push_back(H * U_new_pred * H.transpose());
	Uk_pred_history.push_back(H * (phi * U_new_pred * phi.transpose() + W) * H.transpose());

//	Vector2f meas2d;
//	Matrix2f Cov2d, predCov2d;
//	Vector3f temp3 = H * z_new_pred;
//	meas2d(0) = temp3(0);	meas2d(1) = temp3(1);
//	Matrix3f temp;
//	temp = H * U_new_pred * H.transpose();
//	Cov2d << temp(0,0), temp(0,1),
//		 temp(1,0), temp(1,1);
//	temp = H * (phi * U_new_pred * phi.transpose() + W) * H.transpose();
//	predCov2d << temp(0,0), temp(0,1),
//		     temp(1,0), temp(1,1);
//	ftr_meas_history_2d.push_back(meas2d);
//	ftr_measCov_history_2d.push_back(Cov2d);
//	Uk_pred_history_2d.push_back(predCov2d);
}

#endif
