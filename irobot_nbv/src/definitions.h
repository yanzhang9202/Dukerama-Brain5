#ifndef _DEFINITIONS_H
#define _DEFINITIONS_H

#include <Eigen/Dense>

#define PI 3.14159265
#define NUM_TARGETS 1
#define NUM_MEASUREMENT 10
// Iteration Parameter in Local Controller Level
#define DT 0.001
#define MOVE_DISTANCE 0.05
#define MAX_ITERATIONS 500
// Iteration Parameter in global Controller Level
#define POSE_GAIN 1
#define RK_H 0.001
#define FOV_PENALTY 0.01

float bline;	    // Baseline in mm;
float flength;	    // Focal length in pixel;

Eigen::MatrixXf Hc2m_left(4,4), Hc2m_right(4,4); // Camera to hand transformation matrix
Eigen::VectorXf cc_left(2), cc_right(2);

#endif

//#define BASELINE 0.203316388661
//#define FOCAL_LENGTH 656//667//649//847//830
//#define FRAMERATE 0.01
//#define RK_H 0.001
//#define DT 0.001
//#define FOV_PENALTY 0.01
//#define POSE_GAIN 1
//#define RELATIVE_GAIN 1
//#define MOVE_DISTANCE 0.03
//#define MAX_ITERATIONS 500
//#define NUM_TARGETS 4
//#define ROTATION_GAIN 1
//#define TARGET_DETECT_ERROR 2048


////#define printpos
////#define TEST
////#define printpostarget
////#define printrottarget
//#define printimagecoords
//#define printpk
////#define printvirtualpk
//#define printxk
////#define printnextpk
////#define printtranslvel
////#define printrotvel
////#define nomove
//#define printtofile
////#define printtranslgradients
////#define printrotgradients
////#define SHOWIMAGE
