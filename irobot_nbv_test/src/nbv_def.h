#ifndef ____NBV_DEF_H____
#define ____NBV_DEF_H____

#include <Eigen/Dense>

#define NUM_MEAS 15
#define kf_delta 0.01

// Relative control frame
#define delta 0.2
#define tol 0.00001
#define Dt 0.01 	// Integration time interval for translation
#define gTrade 0.5
#define pen 0.01	// FoV violation penalty
#define h 0.01

struct CircleCtr{
  cv::Mat ctr;
};

struct Objective{
  Eigen::Matrix3f Uk;
  Eigen::Vector3f xk, pk;
  int which_target;
};

#endif
