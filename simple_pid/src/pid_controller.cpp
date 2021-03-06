fndef __PID_CONTROLLER_cpp___
#define __PID_CONTROLLER_cpp___

#include "pid_controller.h"

pid_controller::pid_controller(float Kpp, float Kii, float Kdd, double ull, double lll, double Error_Tolerance)
{
    Kp = Kpp; Ki = Kii; Kd = Kdd; ul = ull; ll = lll; tolerance = Error_Tolerance;
      error = 0;
        prev_time = 0.0;
          delta_t = 0.0;
            error_integral = 0.0;
              cutoff_frequency= -1.0;
                error_list = Eigen::Vector3f::Zero();
                  filtered_error = Eigen::Vector3f::Zero();
                    error_deriv = Eigen::Vector3f::Zero();
                      filtered_error_deriv = Eigen::Vector3f::Zero();
                        loop_counter = 0;
                          anti_w = 0.5;
                            duration = 0;
                              start_time = 0;
}

void pid_controller::setParam(float Kpp, float Kii, float Kdd, double ull, double lll, double Error_Tolerance)
{
    Kp = Kpp; Ki = Kii; Kd = Kdd; ul = ull; ll = lll; tolerance = Error_Tolerance;
}

void pid_controller::clearhistory()
{
    prev_time = 0.0;
      delta_t = 0.0;
        error_integral = 0.0;

          error_list.setZero();
            filtered_error.setZero();
              error_deriv.setZero();
                filtered_error_deriv.setZero();
                  loop_counter = 0;
}

void pid_controller::setError(float EstError)
{
    error = EstError;
}

float pid_controller::computePID()
{
      if (prev_time == 0)
              { prev_time = ros::Time::now().toSec(); }
          else
                  {
                            delta_t = ros::Time::now().toSec() - prev_time;
                                    prev_time = ros::Time::now().toSec();
                                          }

              error_list(2) = error_list(1);
                  error_list(1) = error_list(0);
                      error_list(0) = error;

                      //    std::cout << error << "\n";

                          if (fabs(error) < tolerance) // The robot center is within 5mm range of waypoint
                                {
                                         u = 0;
                                             }
                              else
                                    {
                                            // integrate the error
                                            error_integral += error_list(0)*delta_t;

                                                  // Apply anti-windup to limit the size of the integral term
                                                  if ( error_integral > abs( anti_w ) )
                                                            error_integral = abs(anti_w);

                                                        if ( error_integral < -abs( anti_w ) )
                                                                  error_integral = -abs(anti_w);

                                                              // My filter reference was Julius O. Smith III, Intro. to Digital Filters With Audio Applications.
                                                              float c;
                                                                    if (cutoff_frequency == -1)
                                                                              c = 1.0; // Default to a cut-off frequency at one-fourth of the sampling rate
                                                                          else
                                                                                    c = 1/tan( (cutoff_frequency*6.2832)*delta_t/2 );
                                                                           
                                                                                filtered_error(2) = filtered_error(1);
                                                                                      filtered_error(1) = filtered_error(0); 
                                                                                            filtered_error(0) = (1/(1+c*c+1.414*c))*(error_list(2)+2*error_list(1)+error_list(0)-(2-1.414)*filtered_error(2));

                                                                                                  // Take derivative of error
                                                                                                  // First the raw, unfiltered data:
                                                                                                  error_deriv(2) = error_deriv(1);
                                                                                                        error_deriv(1) = error_deriv(0);
                                                                                                              error_deriv(0) = (error_list(0)-error_list(1))/delta_t;

                                                                                                                    filtered_error_deriv(2) = filtered_error_deriv(1);
                                                                                                                          filtered_error_deriv(1) = filtered_error_deriv(0);

                                                                                                                                if ( loop_counter > 2 ) // Let some data accumulate
                                                                                                                                          filtered_error_deriv(0) = (1/(1+c*c+1.414*c))*(error_deriv(2)+2*error_deriv(1)+error_deriv(0)-(2-1.414)*filtered_error_deriv(2));
                                                                                                                                      else
                                                                                                                                              loop_counter++;

                                                                                                                                            // calculate the control effort
                                                                                                                                            u = Kp * filtered_error(0) + Ki * error_integral + Kd * filtered_error_deriv(0);
                                                                                                                                            //      std::cout << ul << " " << ll << " " << tolerance << "\n";

                                                                                                                                                  // Check saturation limits  
                                                                                                                                                  if (u > ul)
                                                                                                                                                            u = ul;
                                                                                                                                                        if (u < ll)
                                                                                                                                                                  u = ll;
                                                                                                                                                            }

                                  return u;
}

#endif

