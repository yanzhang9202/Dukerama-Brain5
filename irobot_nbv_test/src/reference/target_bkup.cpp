#ifndef _TARGETCPP_
#define _TARGETCPP_

#include <Eigen/Dense>
#include "target.h"
#include <iostream>

using namespace Eigen;
using namespace std;

target::target(float b_in, float f_in){

  initialised = false; // Whether measurement history has been initialized

  b=b_in;
  f=f_in;

  sigma_quant = 1.0; 
  Vector3f variances;
  variances << sigma_quant, sigma_quant, sigma_quant;
  Q = variances.asDiagonal();

  meas_history.reserve(NUM_MEASUREMENT);
  measCov_history.reserve(NUM_MEASUREMENT);
  ftr_meas_history.reserve(NUM_MEASUREMENT);
  ftr_measCov_history.reserve(NUM_MEASUREMENT);

  H = Matrix3f::Zero(); // Measurement matrix
  H(0,0)=1;
  H(1,1)=1;
  H(2,2)=1;

  phi = Matrix3f::Zero(); // System dynamic matrix
  phi(0,0)=1;
  phi(1,1)=1;
  phi(2,2)=1;

  K = Matrix3f::Zero();  // Initialized Kalman Filter Gain

  W = Matrix3f::Zero();  // Process Noise

}

// ********************** Calculate target position in Relative Frame *********************** //

void target::update_local(const float xl_in, const float xr_in,const float y_in)
{
  xl=xl_in;
  xr=xr_in;
  y=y_in;

  if (xl==2048 || xr==2048){
      return;
  }

  std::cout << b << " " << f << std::endl;

  std::cout << xl << " " << xr << " " << y << std::endl;
  
  pk(0)=(b*(xl+xr))/(2*(xl-xr));
  pk(1)=(b*y)/(xl-xr);
  pk(2)=(b*f)/(xl-xr);

  std::cout << "Target position in relative frame: " << pk(0) << " " << pk(1) << " " << pk(2) << std::endl << std::endl;
}

// *********************** Kalman Filter current measurement with current belief ************************ //

void target::update_global(Vector3f x_meas, Matrix3f sigma_meas)
{

  sigma = sigma_meas;
  x_measure = x_meas;

  // Next prediction generated based on previous prediction and measurement
  if (initialised){  // Do Kalman Filtering

    z_old_pred = phi * z_new_pred; // Predite State
    U_old_pred = phi * U_new_pred * phi.transpose() + W; // Prediction Noise
    K = U_old_pred * H.transpose() * ((H * U_old_pred * H.transpose() + sigma_meas).inverse()); // Optimal Kalman Gain
    z_new_pred = z_old_pred + K * (x_meas - H * z_old_pred); // Updated state estimate
    U_new_pred = U_old_pred - K * H * U_old_pred; // Updated State Covariance

  } else {      // First prediction generated based on uncertainty matrix and framerate

    for (int i=0;i<3;i++){
      for (int j=0;j<3;j++){
	U_new_pred(i,j)=sigma_meas(i,j);
      }
    }

    U_old_pred=U_new_pred;

    for (int i=0;i<3;i++){
      z_new_pred(i)=x_meas(i);
    }
    z_old_pred=z_new_pred;

    initialised=true;

  }

  x_new_pred = H * z_new_pred;

}

#endif
