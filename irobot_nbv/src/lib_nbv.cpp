#ifndef ___LIB_NBV_CPP___
#define ___LIB_NBV_CPP___

#include "lib_nbv.h"

using namespace std;
using namespace Eigen;

// ************************************ RAMA_ROBOT **************************************** //
rama_robot::rama_robot()
{
  fx = 24;
  fy = 24;

  disp_raw_img = false;
  disp_thred_img = false;
  draw_circle_ctr = true;

  targets.reserve(NUM_TARGETS);
  sigma_meas.reserve(NUM_TARGETS);
  x_meas.reserve(NUM_TARGETS);

  unknown_noise_std = 0.01;
  unknown_sigma = Matrix3f::Zero();
  unknown_sigma(0,0) = unknown_noise_std * unknown_noise_std;
  unknown_sigma(1,1) = unknown_noise_std * unknown_noise_std;
  unknown_sigma(2,2) = unknown_noise_std * unknown_noise_std;

}

void rama_robot::waitForEnter(){
  char Enter;
  std::cin >> Enter;
}

void rama_robot::loadConfig()
{
  int line, col;
  ifstream file;
  file.open("/home/dukerama/hydro_catkin/src/irobot_nbv/include/Camera_Config/set 6 Oct 26.txt");
    
  if (!file.is_open()){
    cout << "No 'set 6 Oct 26.txt' file in the /include/Camera_Config folder!\n";
    return; 
  }

  char dummy[64];

  for(line = 0; line < 15; line++){

    if (line == 0 || line == 2 || line == 4 || line == 6 || line == 11)
    {  file >> dummy;  }
    
    if (line == 1)
    {  file >> flength; }

    if (line == 3){
       for (col = 0; col < 2; col++){
          file >> cc_left(col);
       }
    }

    if (line == 5){
       for (col = 0; col < 2; col++){
          file >> cc_right(col);
       }
    }

    if (line == 7){
       int mline = 0;
       for (line == 7; line < 11; line++){
          for (col = 0; col < 4; col++){
             file >> Hc2m_left(mline, col);
          }
          mline++;
       }
       line = 10;
     }

    if (line == 12){
       int mline = 0;
       for (line == 12; line < 16; line++){
          for (col = 0; col < 4; col++){
             file >> Hc2m_right(mline, col);
          }
          mline++;
       }
     }

  }
  file.close();

  Eigen::VectorXf temp(4);
  temp = Hc2m_left.col(3) - Hc2m_right.col(3);
  bline = temp.norm()/1000;

  std::cout << "bline: " << bline << "(m)"<< std::endl << std::endl;
  std::cout << "flength: " << flength << std::endl << std::endl;
  std::cout << "cc_left: " << cc_left(0) << " " << cc_left(1) << std::endl << std::endl;
  std::cout << "cc_right: " << cc_right(0) << " " << cc_right(1) << std::endl << std::endl;
  std::cout << "Hc2m_left: " << "\n" << Hc2m_left << std::endl << std::endl;
  std::cout << "Hc2m_right: " << "\n" <<Hc2m_right << std::endl << std::endl;

  for (line = 0; line < 3; line++)
  {
      for (col = 0; col < 3; col++)
      {
          Rot_c2h(line, col) = Hc2m_left(line, col);
      }
  }

  for (line = 0; line < 3; line++)
  {
      ocam_hand(line) = (Hc2m_left(line, 3) + Hc2m_right(line, 3))/2/1000;
  }

  flength = flength/fx;
  std::cout << "flength: " << flength << " by down-sample rate " << 1 << "/" << fx << std::endl << std::endl;

  for (int i=0;i<NUM_TARGETS;i++)
  {
      targets.push_back(new target(bline,flength));
  }

}

// ************************** Update measurement *****************************//

void rama_robot::update_meas()
{
    max_val = 0;
    max_count = 0;

    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    for (int i=0;i<NUM_TARGETS;i++)
    {   
        if (sigma_meas.size()<NUM_TARGETS){
           sigma_meas.push_back(calc_sigma(i,targets[i]->xl,targets[i]->xr,targets[i]->y));  // Calculate single measurement uncertainty matrix;
           x_meas.push_back(Rot_c2w*targets[i]->pk+ocam_world); // Calculate target i's position in global frame;
        } else {
           sigma_meas[i]=calc_sigma(i,targets[i]->xl,targets[i]->xr,targets[i]->y);
           x_meas[i]=Rot_c2w*targets[i]->pk+ocam_world;
        }

        targets[i]->meas_history.push_back(x_meas[i]);
        targets[i]->measCov_history.push_back(sigma_meas[i]);

//        std::cout << "Rot_c2w: \n" << Rot_c2w << std::endl;
//	std::cout << "targets[i]->pk: \n" << targets[i]->pk << std::endl;
//        std::cout << "ocam_world: \n" << ocam_world << std::endl;
        std::cout << "Target " << i+1 << "'s single measurement: \n" << x_meas[i][0] << " " << x_meas[i][1] << " " << x_meas[i][2] << " " << std::endl << std::endl;
        std::cout << "Target " << i+1 << "'s single measurement uncertainty: \n" << sigma_meas[i] << std::endl << std::endl;

        // Kalman filtering using current measurement and measurement history

        targets[i]->update_global(x_meas[i], sigma_meas[i]);
	targets[i]->ftr_meas_history.push_back(targets[i]->z_new_pred);
	targets[i]->ftr_measCov_history.push_back(targets[i]->U_new_pred);

        std::cout << "Target " << i+1 << "'s filtered position: \n" << targets[i]->z_new_pred(0) << " " << targets[i]->z_new_pred(1) << " " << targets[i]->z_new_pred(2) << endl << endl;

        std::cout << "Target " << i+1 << "'s filtered Covariance Matrix: \n" << targets[i]->U_new_pred << endl << endl;

        // Find the target having the largest trace of filtered covariance matrix
        if (targets[i]->U_new_pred.trace() > max_val){
          max_val = targets[i]->U_new_pred.trace();
          max_count = i;
        }

        std::cout << "Target " << max_count+1 << " has the largest trace." << std::endl << std::endl;
    }
}

void rama_robot::update_NBV()
{
    pk_next = calc_next_position(max_count); // Local Controller

    euler_rk_update(); // Global Controller

}

// ************************ Calculate next Robot position and orientation in Global Frame ************************** //
void rama_robot::euler_rk_update(){
  Vector3f vel, position_start, position_new;
  Matrix3f k1, k2, k3, k4;
  int iter = 0;

  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce();

  position_start = ocam_world;
  position_new = ocam_world;
  position_target = ocam_world;
  rotation_target = Rot_c2w;

  while ((position_target - position_start).norm() < MOVE_DISTANCE){
    if(iter > MAX_ITERATIONS){
        std::cout << "Global controller: Exceed MAX_ITERATIONS calculating next robot position and orientation!" << std::endl << std::endl; 
        break;
    }

    if(!ros::ok()) break;

    vel = calc_D_psi_r();
    vel = vel/vel.norm();
    position_new = position_target - POSE_GAIN * DT * vel;

    k1 = - rotation_target * calc_D_psi_R(rotation_target);
    k2 = - (rotation_target + RK_H/2 * k1) * calc_D_psi_R(rotation_target + RK_H/2 * k1);
    k3 = - (rotation_target + RK_H/2 * k2) * calc_D_psi_R(rotation_target + RK_H/2 * k2);
    k4 = - (rotation_target + RK_H * k3) * calc_D_psi_R(rotation_target + RK_H * k3);
    rotation_target = (rotation_target + RK_H/6 * (k1 + 2 * k2 + 2 * k3 + k4));

    position_target = position_new;
    iter++;
  }

  std::cout << "Global controller: Former robot position is \n" << ocam_world(0) << " " << ocam_world(1) << " " << ocam_world(2) << std::endl << std::endl;
  std::cout << "Global controller: Next robot position is \n" << position_target(0) << " " << position_target(1) << " " << position_target(2) << std::endl << std::endl;
  
  std::cout << "Global controller: Former robot orientation is \n" << Rot_c2w << std::endl << std::endl;
  std::cout << "Global controller: Next robot orientation is \n" << rotation_target << std::endl << std::endl;

}

// Position gradient
Vector3f rama_robot::calc_D_psi_r(){
  Vector3f D_psi_r, fov_sum;
  Vector3f vel;

  fov_sum = Vector3f::Zero();

  D_psi_r = 2 * (rotation_target * pk_next + position_target - targets[max_count]->x_new_pred);

  for (int i=0;i<NUM_TARGETS;i++){
    for (int j=0;j<3;j++){
      fov_sum = fov_sum + calc_fov_grad_r(i,j);
    }
  }
  
  if (D_psi_r.norm() < (fov_sum.norm() * FOV_PENALTY/NUM_TARGETS)){
     transl_fov = true;  
     std::cout << "Transl fov took over gradient\n" << std::endl;
  }
  
  D_psi_r = D_psi_r + FOV_PENALTY/NUM_TARGETS * fov_sum;

  return D_psi_r;
}

// FOV Constraint for position                                   //  I don't understand what is w !!! This could be problematic !!!
Vector3f rama_robot::calc_fov_grad_r(const int i,const int j){ 
  float phi;
  Vector3f grad_r_phi, pk;
  int w = 1200; float b = bline, f = flength;
  float Dphix_dx, Dphix_dz, Dphiy_dy, Dphiy_dz, Dphiz_dz;
  
  Vector3f grad_r_x(-rotation_target(0,0),-rotation_target(1,0),-rotation_target(2,0));
  Vector3f grad_r_y(-rotation_target(0,1),-rotation_target(1,1),-rotation_target(2,1));
  Vector3f grad_r_z(-rotation_target(0,2),-rotation_target(1,2),-rotation_target(2,2));

  pk = rotation_target.transpose() * (targets[i]->x_measure - position_target);

  Dphix_dx = -2 * pk(0); Dphix_dz = ((w * w)/(2 * f * f)) * pk(2) - w * b * f/(2 * f * f);
  Dphiy_dy = -2 * pk(1); Dphiy_dz = ((w * w)/(2 * f * f)) * pk(2);
  Dphiz_dz = 2 * pk(2);

  if (j == 0) {
    phi =  pow((w*pk(2) - b*f)/(2*f),2) - pk(0)*pk(0);
    grad_r_phi = Dphix_dx * grad_r_x + Dphix_dz * grad_r_z;
    //grad_r_phi=(-2*targets[i]->pk_next(0))*grad_r_x + (w*w*targets[i]->pk_next(2)/f-w*b)*grad_r_z;
  }
  else if (j == 1) {
    phi = pow((w*pk(2))/(2*f),2) - pk(1)*pk(1);
    grad_r_phi = Dphiy_dy * grad_r_y + Dphiy_dz * grad_r_z;
    //grad_r_phi=(-2*targets[i]->pk_next(1))*grad_r_y + (w*w*targets[i]->pk_next(2)/f)*grad_r_z;
  }
  else if (j == 2) {
    phi = pk(2) * pk(2) - pow(b*f/w,2);
    grad_r_phi = /*Dphix_dz*grad_r_x+Dphiy_dz*grad_r_y+*/Dphiz_dz*grad_r_z;
    //grad_r_phi=(2*targets[i]->pk_next(2))*grad_r_z;
  }

  grad_r_phi = -grad_r_phi/(phi*phi);

  return grad_r_phi;
}

// Rotation gradient
Matrix3f rama_robot::calc_D_psi_R(const Matrix3f rotation){
  Matrix3f D_psi_R, fov_sum, D_temp;

  fov_sum = Matrix3f::Zero();
  D_psi_R = - (rotation.transpose() * (position_target - targets[max_count]->x_new_pred) * pk_next.transpose() - pk_next * (position_target - targets[max_count]->x_new_pred).transpose() * rotation);
  /*D_temp=-rotation.transpose()*(position_target-targets[max_count]->x_new_pred)*pk_next.transpose();
    D_psi_R=D_temp-D_temp.transpose();*/

  for (int i=0;i<NUM_TARGETS;i++){
    for (int j=0;j<3;j++){
      fov_sum = fov_sum + calc_fov_grad_R(i,j,rotation);
    }
  }

  if (D_psi_R.norm() < (fov_sum.norm() * FOV_PENALTY/NUM_TARGETS)){
     rot_fov=true;
     std::cout << "Rotational fov took over gradient\n" << std::endl;
  }

  rot_norm = D_psi_R.norm();
  rot_fov_norm = fov_sum.norm() * FOV_PENALTY/NUM_TARGETS;

  D_psi_R = D_psi_R - FOV_PENALTY/NUM_TARGETS * fov_sum;

  return D_psi_R;
}

// FOV Constraint for rotation
Matrix3f rama_robot::calc_fov_grad_R(const int i,const int j, const Matrix3f rotation){
  float phi;
  Vector3f pk;
  Matrix3f grad_R_phi;
  int w = 1200; float b = bline, f = flength;
  Matrix<float,3,1> e1,e2,e3;
  e1 << 1,0,0;
  e2 << 0,1,0;
  e3 << 0,0,1;

  Matrix3f grad_R_x, grad_R_y, grad_R_z;
  Vector3f grad_temp;

  float Dphix_dx, Dphix_dz, Dphiy_dy, Dphiy_dz, Dphiz_dz;
  
  pk = rotation_target.transpose() * (targets[i]->x_measure - position_target);

  grad_temp = - rotation.transpose()/2 * (targets[i]->x_measure - position_target);
  
  grad_R_x=0.5*(-e1*(targets[i]->x_measure-position_target).transpose()*rotation+rotation.transpose()*(targets[i]->x_measure-position_target)*e1.transpose());
  grad_R_y=0.5*(-e2*(targets[i]->x_measure-position_target).transpose()*rotation+rotation.transpose()*(targets[i]->x_measure-position_target)*e2.transpose());
  grad_R_z=0.5*(-e3*(targets[i]->x_measure-position_target).transpose()*rotation+rotation.transpose()*(targets[i]->x_measure-position_target)*e3.transpose());

  /*grad_R_x=grad_temp*e1.transpose()-e1*grad_temp.transpose();
  grad_R_y=grad_temp*e2.transpose()-e2*grad_temp.transpose();
  grad_R_z=grad_temp*e3.transpose()-e3*grad_temp.transpose();*/

  Dphix_dx = -2 * pk(0); Dphix_dz = ((w * w)/(2 * f * f)) * pk(2) - w * b * f/(2 * f * f);
  Dphiy_dy = -2 * pk(1); Dphiy_dz = ((w * w)/(2 * f * f)) * pk(2);
  Dphiz_dz = 2 * pk(2);

  if (j == 0) {
    phi = pow((w * pk(2) - b*f)/(2 * f),2) - pk(0) * pk(0);
    grad_R_phi = Dphix_dx * grad_R_x + Dphix_dz * grad_R_z;
    //grad_r_phi=(-2*pk_next(0))*grad_r_x + (w*w*pk_next(2)/f-w*b)*grad_r_z;                                                                                                                
  }
  else if (j == 1) {
    phi = pow((w * pk(2))/(2 * f),2) - pk(1) * pk(1);
    grad_R_phi = Dphiy_dy * grad_R_y + Dphiy_dz * grad_R_z;
    //grad_r_phi=(-2*pk_next(1))*grad_r_y + (w*w*pk_next(2)/f)*grad_r_z;                                                                                                                            
  }
  else if (j == 2) {
    phi = pk(2) * pk(2) - pow(b*f/w,2);
    grad_R_phi = /*Dphix_dz*grad_R_x+Dphiy_dz*grad_R_y+*/Dphiz_dz*grad_R_z;
    //grad_r_phi=(2*pk_next(2))*grad_r_z;                                                                                                                                                           
  }

  grad_R_phi=-grad_R_phi/(phi*phi);

  return grad_R_phi;
}

// ************************ Calculate next target position in Relative Camera Frame ************************* //
Eigen::Vector3f rama_robot::calc_next_position(int target_num)
{
  int count = 0;
  
  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce();

  Vector3f p_temp = Rot_w2c * (targets[target_num]->x_new_pred - ocam_world);//targets[target_num]->pk;
  Vector3f p_old = p_temp;//targets[target_num]->pk;
  Vector3f gradient;

  while((p_temp-p_old).norm() < MOVE_DISTANCE){
    update_dJQJT_dj(p_temp, target_num);
    gradient = calc_gradient(target_num, p_temp);
    p_temp = p_temp - DT * gradient;
    count++;

    if (count >= MAX_ITERATIONS){
	std::cout << "Local controller: Exceed MAX_ITERATIONS calculating next target relative position!" << std::endl << std::endl;
        break;
    }
  }

  std::cout << "Former target position in Relative Camera Frame: " << p_old(0) << " " << p_old(1) << " " << p_old(2) << std::endl << std::endl;
  std::cout << "Next target position in Relative Camera Frame: " << p_temp(0) << " " << p_temp(1) << " " << p_temp(2) << std::endl << std::endl;

  return p_temp;
}

// ************************ Caculate dh/d(pk)_x_y_z in local controller ************************ //
void rama_robot::update_dJQJT_dj(Vector3f pk,int target_num){    // Could be problematic, check the derivation process if needed !!!

  Matrix3f temp,DU_Dpx,DU_Dpy,DU_Dpz;
  float f = flength;
  float b = bline;
  float xl = f * (pk(0) + b/2) / pk(2);
  float xr = f * (pk(0) - b/2) / pk(2);
  float y = pk(1) * f / pk(2);
  Matrix3f J = calc_J(xl, xr, y);
  //std::cout << "J is:\n" << J << std::endl;
  Matrix3f JQJT = J * targets[target_num]->Q * J.transpose();
//  double ol2=targets[target_num]->ol2;
//  double or2=targets[target_num]->or2;
//  double oy2=targets[target_num]->oy2;
  float sigma_quant = targets[target_num]->sigma_quant;
  float base = -1 * pow(b,2)/pow(xl-xr,5);
  float sum1 = 2 * sigma_quant;
  float sum2 = 4 * sigma_quant * xr + sigma_quant * (3 * xl + xr);
  float dsp=xl-xr;
    
  float DxL_Dpx = f / pk(2), DxL_Dpy=0, DxL_Dpz = - f * (pk(0) + b/2) / (pk(2) * pk(2));
  float DxR_Dpx = f / pk(2), DxR_Dpy=0, DxR_Dpz = - f * (pk(0) - b/2) / (pk(2) * pk(2));
  float Dy_Dpx = 0, Dy_Dpy = f / pk(2), Dy_Dpz = - pk(1) * f / (pk(2) * pk(2));
    
  float beta = pow(dsp, 4);
  float Dbeta_DxR = - 4 * pow(dsp, 3);
  float Dbeta_DxL = 4 * pow(dsp, 3);
  float Dbeta_Dy = 0;
    
  Matrix3f DU_DxL;
  DU_DxL << 2 * b * b * sigma_quant * xr, b * b * y * sigma_quant, b * b * f * sigma_quant, b * b * y * sigma_quant, 2 * b * b * dsp * sigma_quant, b * b * f * sigma_quant, b * b * f * sigma_quant, b * b * f * sigma_quant, 0;                                                        DU_DxL=DU_DxL/beta-JQJT/beta*Dbeta_DxL;
  DU_DxL = DU_DxL / beta - JQJT / beta * Dbeta_DxL;
  Matrix3f DU_DxR;
  DU_DxR << 2 * b * b * sigma_quant * xr, b * b * y * sigma_quant, b * b * f * sigma_quant, b * b * y * sigma_quant, - 2 * b * b * dsp * sigma_quant, - b * b * f * sigma_quant, b * b * f * sigma_quant, - b * b * f * sigma_quant,0;
  DU_DxR = DU_DxR / beta - JQJT / beta * Dbeta_DxR;
  Matrix3f DU_Dy;
  DU_Dy << 0, b * b * (xr * sigma_quant + xl * sigma_quant), 0, b * b * (xr * sigma_quant + xl * sigma_quant), 2 * b * b * y * 2 * sigma_quant, b * b * f * sigma_quant, 0, b * b * f * sigma_quant, 0;
  DU_Dy = DU_Dy / beta - JQJT / beta * Dbeta_Dy;
    
  dJQJT_dxl = DU_DxL;
  dJQJT_dxr = DU_DxR;
  dJQJT_dy = DU_Dy;
}

// Calculate the derivative of the relative next best view with respect to the relative position of the targets
float rama_robot::calc_dh_dpj(int target_num,const Vector3f d_dpj, Vector3f pk){
  float f = flength;
  float b = bline;
  float xl = f * (pk(0) + b/2) / pk(2);
  float xr = f * (pk(0) - b/2) / pk(2);
  float y = pk(1) * f / pk(2);
  Matrix3f J = calc_J(xl, xr, y);
  Matrix3f JQJT = J * targets[target_num]->Q * J.transpose();
  Matrix3f xi = calc_xi(target_num, xl, xr, y);
  float dh_dpj = (JQJT.inverse() * xi * xi * JQJT.inverse() * (dJQJT_dxl * d_dpj(0) + dJQJT_dxr * d_dpj(1) + dJQJT_dy * d_dpj(2))).trace();
  return dh_dpj;
}

Matrix3f rama_robot::calc_xi(int target_num, float xl, float xr, float y){
  Matrix3f J=calc_J(xl, xr, y);
  Matrix3f H=targets[target_num]->H;
  Matrix3f xi = ((Rot_c2w.transpose() * H * targets[target_num]->U_new_pred * H.transpose() * Rot_c2w).inverse() + ( J * targets[target_num]->Q * J.transpose()).inverse()).inverse();
  return xi;
}

// Calculate relative next best view gradient
Eigen::Vector3f rama_robot::calc_gradient(int target_num, const Vector3f & p_old){
  Vector3f gradient;
  Vector3f d_dpx, d_dpy, d_dpz;
  d_dpx << flength / p_old(2), flength / p_old(2), 0;
  d_dpy << 0, 0, flength / p_old(2);
  d_dpz << -flength * (p_old(0) + bline / 2) / pow(p_old(2),2), -flength * (p_old(0) - bline/2) / pow(p_old(2),2), -flength * p_old(1) / pow(p_old(2),2);

  gradient(0) = calc_dh_dpj(target_num, d_dpx, p_old);
  gradient(1) = calc_dh_dpj(target_num, d_dpy, p_old);
  gradient(2) = calc_dh_dpj(target_num, d_dpz, p_old);

  gradient = gradient/gradient.norm();

  return gradient;
}

// ************************ Single measurement uncertainty *************************** //
// Calculate the measurement uncertainty
Matrix3f rama_robot::calc_J(float xl, float xr, float y){
  float disp=xl-xr;
  Matrix3f J;
  J << -bline*xr,bline*xl,0,-bline*y,bline*y,bline*disp,-bline*flength,bline*flength,0;
  J=J/(disp*disp);
  return J;
}

Matrix3f rama_robot::calc_sigma(int target_num, float xl, float xr, float y){
  Matrix3f J = calc_J(xl,xr,y);
  Matrix3f sigma_meas = Rot_c2w * J * targets[target_num]->Q * J.transpose() * Rot_c2w.transpose();
//  std::cout << "Rotation determinant is: " << Rot_c2w.determinant() << std::endl << std::endl;
  return sigma_meas + unknown_sigma;
}

// ************************** Circle detector *************************** //

void rama_robot::detectCircle(cv::Mat image_left, cv::Mat image_right)
{
    cv::Mat img_down_left, img_down_right;
    img_down_left = downsample(image_left);
    img_down_right = downsample(image_right);

//    cv::Mat img_thre_left, img_thre_right;
//    img_thre_left = robot.findCircle(img_down_left);
//    img_thre_right = robot.findCircle(img_down_right);

    for (int i=0; i < NUM_TARGETS; i++)
    {
        ctr_left = findCircle(img_down_left);
        ctr_right = findCircle(img_down_right);

        xl = (float)ctr_left.ctr.at<uchar>(0,0) - cc_left(0)/fx;
        xr = (float)ctr_right.ctr.at<uchar>(0,0) - cc_right(0)/fx;
        yl = (float)ctr_left.ctr.at<uchar>(0,1) - cc_left(1)/fy;
        yr = (float)ctr_right.ctr.at<uchar>(0,1) - cc_right(1)/fy;
//        std::cout << xl << " " << xr << " " << yl << " " << yr << std::endl;

        targets[i]->update_local(xl,xr,yl);  // Calculate target i's position in relative camera frame; 
    }

    if (disp_raw_img){
       //show image to image window
       imshow("left",image_left);
       cv::waitKey(30);
       imshow("right",image_right);
       cv::waitKey(30);
    }

    if (disp_thred_img){
       cv::namedWindow("proc_right",CV_WINDOW_NORMAL);
       cv::namedWindow("proc_left",CV_WINDOW_NORMAL);

       imshow("proc_left",ctr_left.img_thred);
       cv::waitKey(30);
       imshow("proc_right",ctr_right.img_thred);
       cv::waitKey(30);

       imshow("left",image_left);
       cv::waitKey(30);
       imshow("right",image_right);
       cv::waitKey(30);
    }

    if (draw_circle_ctr)
    {
        cv::namedWindow("ctr_right",CV_WINDOW_NORMAL);
        cv::namedWindow("ctr_left",CV_WINDOW_NORMAL);

        std::cout << "Left circle center: " << ctr_left.ctr << std::endl << std::endl;
        std::cout << "Right circle center: " << ctr_right.ctr << std::endl << std::endl;

       	cv::Point center_left((int)ctr_left.ctr.at<uchar>(0,0), (int)ctr_left.ctr.at<uchar>(0,1));     
	circle( img_down_left, center_left, 1, cv::Scalar(0, 0, 255), -1, 8, 0 );
       	cv::Point center_right((int)ctr_right.ctr.at<uchar>(0,0), (int)ctr_right.ctr.at<uchar>(0,1));
	circle( img_down_right, center_right, 1, cv::Scalar(0, 0, 255), -1, 8, 0 );

        imshow("ctr_left",img_down_left);
        cv::waitKey(30);
        imshow("ctr_right",img_down_right);
        cv::waitKey(30);

        imshow("left",image_left);
        cv::waitKey(30);
        imshow("right",image_right);
        cv::waitKey(30);
    }
}

cv::Mat rama_robot::downsample(cv::Mat image)
{
  cv::Size Size = image.size();
  int rows = Size.height;
  int cols = Size.width;
  cv::Mat new_img;
  cv::Size newSize(cols/fx, rows/fy);
  resize(image, new_img, newSize);
  return new_img;
}

CircleCtr rama_robot::findCircle(cv::Mat image)
{
  cv::Mat imgHSV;
  cv::cvtColor(image, imgHSV, cv::COLOR_BGR2HSV);
  cv::Mat imgThresholded;

//  Orange_MIN = [6, 50, 50], Orange_MAX = [14, 255, 255]
  inRange(imgHSV, cv::Scalar(6, 50, 50), cv::Scalar(14, 255, 255), imgThresholded);
//  Some Morphological opening stuff, removing small objects
  erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
  dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
//  Some Morphological opening stuff, filling small holes
  dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2,2)));
  erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2,2)));

  cv::Size size = image.size();
  cv::Mat mask = cv::Mat::zeros(size.height, size.width, CV_8U);
  mask(cv::Rect(0, size.height/2, size.width, size.height-size.height/2)) = 1;
  cv::Mat img_mask;
  imgThresholded.copyTo(img_mask, mask);
//  std::cout << size.height/3 << " " << size.height - size.height/3 << std::endl;

  cv::Mat nonZeros, nZ_converted;
  cv::findNonZero(img_mask, nonZeros);
  nonZeros.convertTo(nZ_converted, CV_8UC1);

//  std::cout << "Non zero Locations: " << nonZeros << std::endl;

  cv::Mat row_mean(cv::Mat::zeros(1,2,CV_32SC1));
  cv::reduce(nZ_converted, row_mean, 0, CV_REDUCE_AVG);

//  std::cout << "Detected center: " << row_mean << std::endl; 

  CircleCtr result;

  result.img_thred = imgThresholded;
  result.img_mask = img_mask;
  result.ctr = row_mean;

//  return imgThresholded;
//  return outputimg;
  return result;
}

// *********************** VRPN ************************ //
void vrpnCallback(const geometry_msgs::TransformStamped msg){
  {
    q_h[0] =msg.transform.rotation.x;
    q_h[1] =msg.transform.rotation.y;
    q_h[2] =msg.transform.rotation.z;
    q_h[3] =msg.transform.rotation.w;
  
    Eigen::Quaternionf quat_h(q_h[3], q_h[0], q_h[1], q_h[2]);

    Rot_h2w = quat_h.matrix();
    Rot_c2w = Rot_h2w * Rot_c2h;
    Rot_w2c = Rot_c2w.transpose();

    ohand_world[0] = msg.transform.translation.x;
    ohand_world[1] = msg.transform.translation.y;
    ohand_world[2] = msg.transform.translation.z;

    ocam_world = Rot_h2w * ocam_hand + ohand_world;
  }
}

#endif

