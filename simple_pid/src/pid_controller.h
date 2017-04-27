fndef ___PID_CONTROLLER_H____
#define ___PID_CONTROLLER_H____

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>

class pid_controller
{
    public:
          pid_controller(float, float, float, double, double, double);
              void clearhistory();
                  void setError(float EstError);
                      float computePID();
                          void setParam(float Kpp, float Kii, float Kdd, double ull, double lll, double Error_Tolerance);

                            protected:
                              float error;
                                  float Kp;
                                      float Ki;
                                          float Kd;
                                              double prev_time;
                                                  float delta_t;
                                                      float error_integral;
                                                          float cutoff_frequency;
                                                              float anti_w;
                                                                  double ul, ll;
                                                                      Eigen::VectorXf error_list;
                                                                          Eigen::VectorXf filtered_error;
                                                                              Eigen::VectorXf error_deriv;
                                                                                  Eigen::VectorXf filtered_error_deriv;
                                                                                      int loop_counter;
                                                                                          double tolerance;
                                                                                              float u;
                                                                                                  float duration;
                                                                                                      float start_time;
};

#endif

