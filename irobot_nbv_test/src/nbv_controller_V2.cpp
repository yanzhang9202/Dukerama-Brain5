#ifndef ___NBV_CONTROLLER_V2_CPP___
#define ___NBV_CONTROLLER_V2_CPP___

#include "nbv_controller_V2.h"

using namespace Eigen;
using namespace std;

nbv_controller::nbv_controller(ros::NodeHandle &nh, int num_t, int ds, string obj, bool simu, int dimen):
outfile_pose("/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/positions/handpose.txt") {
	downsample = (float)ds;
	cout << "Initialize controller..." << endl;
	getCameraCalib();
	getCameraUncertainty();
	
	num_targets = num_t;
	targets.reserve(num_targets);
	for (int i = 0; i < num_targets; i++){
		targets.push_back(new target());
	}

	vrpn_sub = nh.subscribe<geometry_msgs::TransformStamped>("Brain5Camera/pose", 1, &nbv_controller::vrpnCallback, this);

	supreme = "supreme";
	central = "central";
	choice = obj;
	count_meas = 0;

	it_max = (int)(1/Dt);
//	relgains << 1, 0, 0, 0, 1, 0, 0, 0, 5 ;
	relgains << 1, 0, 0, 1;
	val = 0;
	dh << 0, 0;

	phi.resize(2, num_targets);
	pen_phi.resize(2, num_targets);
	p.resize(2, num_targets);

//	dphix_dpx = Matrix<float, 1, num_targets>::Zero();
//	dphix_dpy = Matrix<float, 1, num_targets>::Zero();
//	dphix_dpz = Matrix<float, 1, num_targets>::Zero();
//	dphiy_dpx = Matrix<float, 1, num_targets>::Zero();
//	dphiy_dpy = Matrix<float, 1, num_targets>::Zero();
//	dphiy_dpz = Matrix<float, 1, num_targets>::Zero();
//	dphiz_dpx = Matrix<float, 1, num_targets>::Zero();
//	dphiz_dpy = Matrix<float, 1, num_targets>::Zero();
//	dphiz_dpz = Matrix<float, 1, num_targets>::Zero();

//	dpen_phi = Matrix<float, 3, num_targets>::Zero();

	dphix_dpx.resize(1, num_targets); dphix_dpx.setZero();
	dphix_dpy.resize(1, num_targets); dphix_dpy.setZero();
	dphix_dpz.resize(1, num_targets); dphix_dpz.setZero();
	dphiy_dpx.resize(1, num_targets); dphiy_dpx.setZero();
	dphiy_dpy.resize(1, num_targets); dphiy_dpy.setZero();
	dphiy_dpz.resize(1, num_targets); dphiy_dpz.setZero();
	dphiz_dpx.resize(1, num_targets); dphiz_dpx.setZero();
	dphiz_dpy.resize(1, num_targets); dphiz_dpy.setZero();
	dphiz_dpz.resize(1, num_targets); dphiz_dpz.setZero();

	dpen_phi.resize(2, num_targets); dpen_phi.setZero();

	e1 << 1, 0;	e2 << 0, 1;
	
	dphix_dr.resize(2, num_targets); dphix_dr.setZero();
	dphiz_dr.resize(2, num_targets); dphiz_dr.setZero();

	dpx_dR.reserve(num_targets);
	dpy_dR.reserve(num_targets);
	dpz_dR.reserve(num_targets);
	
	z.reserve(num_targets);
	zi.reserve(num_targets);

	dphix_dR.reserve(num_targets);
	dphiy_dR.reserve(num_targets);
	dphiz_dR.reserve(num_targets);

	penGrad = Matrix2f::Zero();

	// Simulation
	sim = simu;

	dim = dimen;

	if (sim) {
		bline = 1.0;
		wi = 1024.0;
		fov = 70 * (PI / 180);
		alphaq = tan(fov / 2);
		c = 1.0/(2 * alphaq);
		flength = (0.5 * wi) / alphaq;
	} else {
		wi = 54;
		fov = 60 * (PI / 180);
		alphaq = tan(fov / 2);
		c = 1.0/(2 * alphaq);
	}	

	// Image width setting (for FoV constraint)
	cout << "FoV constraint: width of image is set to be " << wi << "..." << endl << endl;

	show_iter = false;
}

void nbv_controller::calcNBV(std::vector<target *> input, std::vector<bool> flag){

//	// Show current pose
//	cout << "Current pose: " << endl;
//	cout << "r: " << r_sim_2d.transpose() << endl;
//	cout << "R: " << endl << R_sim_2d << endl << endl;

	// Update target information
	getUpdate(input);

	// Find objective target
	findObjective(flag);
	cout << "The objective target is " << objective.which_target << "!" << endl << endl;

	if (dim == 2) {
		Vector2f p_star;
		// Update NBV in relative frame
		p_star = getNBVinRF();
		// Update NBV in global frame
		getNBVinGF(p_star);
	} else if (dim == 3) {
	
	} else { cout << "Error: wrong dimension!" << endl << endl; }

	// Show nbv pose
	cout << "NBV pose: " << endl;
	cout << "r: " << r_new.transpose() << endl;
	cout << "R: " << endl << R_new << endl << endl;

	calcOrder();

	count_meas++;
}

void nbv_controller::recordPose(){
	ros::spinOnce();
  	for (int i=0;i<3;i++) {
    		outfile_pose << ohand_world(i) << " ";
  	}
  	for (int i=0;i<4;i++) {
    		outfile_pose << q_h(i) << " ";
  	}
  	outfile_pose << "\n";
	if (count_meas == NUM_MEAS){
		outfile_pose.close();
	}
}

// ********************** Private *********************** //
void nbv_controller::getUpdate(std::vector<target *> input){
	targets = input;
	ros::spinOnce();
	for (int ii = 0; ii < num_targets; ii++){
//		p.col(ii) = getRelativePosition(targets[ii]->ftr_meas_history[count_meas]);
		z[ii] = targets[ii]->ftr_meas_history[count_meas];
		zi[ii](0) = z[ii](0);	zi[ii](1) = z[ii](1);
	}
}

void nbv_controller::vrpnCallback(const geometry_msgs::TransformStamped msg){
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

void nbv_controller::getCameraCalib(){
	int line, col;
	ifstream file;
  	file.open("/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/Camera_Config/set 6 Oct 26.txt");
    
  	if (!file.is_open()){
    		cout << "No 'set 6 Oct 26.txt' file in the /include/Camera_Config folder!\n";
    		return; 
  	}

  	char dummy[64];

  	for(line = 0; line < 15; line++){
    		if (line == 0 || line == 2 || line == 4 || line == 6 || line == 11) {  file >> dummy; }
    
    		if (line == 1) {  file >> flength; }

    		if (line == 3){
       			for (col = 0; col < 2; col++) { file >> cc_left(col); }
    		}

    		if (line == 5){
       			for (col = 0; col < 2; col++) { file >> cc_right(col); }
    		}

    		if (line == 7){
       			int mline = 0;
       			for (line == 7; line < 11; line++){
          			for (col = 0; col < 4; col++) { file >> Hc2m_left(mline, col); }
          			mline++;
       			}
       			line = 10;
     		}

    		if (line == 12){
       			int mline = 0;
       			for (line == 12; line < 16; line++){
          			for (col = 0; col < 4; col++) { file >> Hc2m_right(mline, col); }
         			mline++;
       			}
     		}
  	}
  	file.close();

  	for (line = 0; line < 3; line++){
      		for (col = 0; col < 3; col++){
          		Rot_c2h(line, col) = Hc2m_left(line, col);
      		}
  	}

  	VectorXf temp(4);
  	temp = Hc2m_left.col(3) - Hc2m_right.col(3);
  	bline = temp.norm()/1000;

//  	cout << "bline: " << bline << "(m)"<< endl << endl;
//  	cout << "flength: " << flength << "(pixel)" << endl << endl;
//  	cout << "cc_left: " << cc_left(0) << " " << cc_left(1) << endl << endl;
//  	cout << "cc_right: " << cc_right(0) << " " << cc_right(1) << endl << endl;
//  	cout << "Hc2m_left: " << "\n" << Hc2m_left << endl << endl;
//  	cout << "Hc2m_right: " << "\n" << Hc2m_right << endl << endl;

  	for (line = 0; line < 3; line++){
      		ocam_hand(line) = (Hc2m_left(line, 3) + Hc2m_right(line, 3))/2/1000;
  	}

  	flength = flength/downsample;
//  	cout << "flength: " << flength << " by down-sample rate " << downsample << endl << endl;
	cc_left = cc_left/downsample;	cc_right = cc_right/downsample;
//  	cout << "cc_left: " << cc_left(0) << " " << cc_left(1) << " by down-sample rate " << downsample << endl << endl;
//  	cout << "cc_right: " << cc_right(0) << " " << cc_right(1) << " by down-sample rate " << downsample << endl << endl;		
}

void nbv_controller::getCameraUncertainty(){
	int line, col;
	ifstream file;
  	file.open("/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/Camera_Config/Q.txt");
    
  	if (!file.is_open()){
    		cout << "No 'Q' file in the /include/Camera_Config folder!\n";
    		return; 
  	}

	for (line = 0; line < 3; line++){
		for (col = 0; col < 3; col++){
			file >> Q3d(line, col);	
		}
	}
	file.close();
	Q = matrix2d(1,2,Q3d);
//	cout << "Uncertainty matrix on image plane: " << endl;
//	cout << Q << endl;
}

Eigen::Vector2f nbv_controller::getRelativePosition(Eigen::Matrix2f R, Eigen::Vector2f r, Eigen::Vector3f global){
	Eigen::Vector2f glob_2d, relative;
	glob_2d(0) = global(0);	glob_2d(1) = global(1);
	relative = R.transpose() * (glob_2d - r);
	return relative;
}

// ********************* Find objectives ********************* //
void nbv_controller::findObjective(std::vector<bool> flag){
	ros::spinOnce();
	if (choice.compare(supreme) == 0){
		cout << "You choose supreme objective for NBV!" << endl << endl;
		if (dim == 3) {
			float max_trace = targets[0]->ftr_measCov_history[count_meas].trace();
			which_target = 0;
	//		cout << "num_targets: " << num_targets << endl;
	//		cout << targets[0]->ftr_measCov_history[count_meas].trace() << endl;
			for (int i = 0; i < num_targets; i++){
				if (targets[i]->ftr_measCov_history[count_meas].trace() > max_trace){
					max_trace = targets[i]->ftr_measCov_history[count_meas].trace();
					which_target = i;
				}
			}
//			objective.Uk = targets[which_target]->Uk_pred_history[count_meas];	// This is the correct version
			objective.Uk = targets[which_target]->ftr_measCov_history[count_meas];
			objective.xk = targets[which_target]->ftr_meas_history[count_meas];
//			objective.pk = getRelativePosition(R_sim_2d, r_sim_2d, objective.xk);
			objective.which_target = which_target + 1;
		} else if (dim == 2) {
			Eigen::Matrix2f temp2;
			Eigen::Matrix3f temp3;
			float max_trace = 0;
			for (int i = 0; i < num_targets; i++) {
				if (targets[i]->ftr_meas_history[count_meas].norm() > std::numeric_limits<float>::epsilon()){				
 					temp3 = targets[i]->ftr_measCov_history[count_meas];
					temp2 = matrix2d(1,2,temp3);
					max_trace = temp2.trace();
					which_target = i;
					break;
				}
			}
			if (count_meas == 0){
				for (int i = 0; i < num_targets; i++){
					if (flag[i]){
						temp2 = matrix2d(1,2,targets[i]->ftr_measCov_history[count_meas]);
						if (temp2.trace() > max_trace){
							max_trace = temp2.trace();
							which_target = i;
						}
					}
				}
			} else {
				for (int i = 0; i < num_targets; i++){
					if (targets[i]->ftr_meas_history[count_meas].norm() > std::numeric_limits<float>::epsilon()){
						temp2 = matrix2d(1,2,targets[i]->ftr_measCov_history[count_meas]);
						if (temp2.trace() > max_trace){
							max_trace = temp2.trace();
							which_target = i;
						}
					}
				}
			}
//			objective.Uk = targets[which_target]->Uk_pred_history[count_meas];	// This is the correct version
			objective.Uk = targets[which_target]->ftr_measCov_history[count_meas];
			objective.xk = targets[which_target]->ftr_meas_history[count_meas];
//			objective.pk = getRelativePosition(R_sim_2d, r_sim_2d, objective.xk);
			objective.which_target = which_target + 1;
		} else { cout << "Error: Wrong dimension " << dim << "!" << endl << endl; }

	} else if (choice.compare(central) == 0){
		cout << "You choose central objective for NBV!" << endl << endl;
//		Matrix3f sum; sum = Matrix3f::Zero();
//		Vector2f ssum; ssum = Vector2f::Zero();
//		Vector3f sssum; sssum = Vector3f::Zero();
//		for (int i = 0; i < num_targets; i++){
//			sum = sum + targets[i]->Uk_pred_history[count_meas];
//			ssum = ssum + getRelativePosition(R_sim_2d, r_sim_2d, targets[i]->ftr_meas_history[count_meas]);
//			sssum = sssum + targets[i]->ftr_meas_history[count_meas];
//		}
//		objective.Uk = sum/(float)num_targets;
//		objective.xk = sssum/(float)num_targets;
////		objective.pk = ssum/(float)num_targets;
//		objective.which_target = -1;
//
		Matrix3f msum; msum = Matrix3f::Zero();
		Vector3f vsum; vsum = Vector3f::Zero();
		int tcount = 0;
		if (dim == 2){
			for (int i = 0; i < num_targets; i++){
				if (targets[i]->ftr_meas_history[count_meas].norm() > std::numeric_limits<float>::epsilon()){
					msum = msum + targets[i]->ftr_measCov_history[count_meas];
					vsum = vsum + targets[i]->ftr_meas_history[count_meas];
				}
				tcount++;
			}
			objective.Uk = msum / (float)tcount;
			objective.xk = vsum / (float)tcount;
			objective.which_target = -1;			
		} else {
			cout << " Dim 3 hasn't been dealt with! " << endl << endl;
		}		
	} else {
		cout << "Error: Illegal objective choice!" << endl << endl;
	}
}

// ******************* NBV control in Relative Frame ********************** //
void nbv_controller::getSimPose(Eigen::Vector2f r_input, Eigen::Matrix2f R_input){
	r_sim_2d = r_input;
	R_sim_2d = R_input;
	Rot_c2w << R_sim_2d(0, 0), 0, R_sim_2d(0, 1),
	           R_sim_2d(1, 0), 0, R_sim_2d(1, 1),
		   0,	 	  -1, 0;
	Rot_w2c = Rot_c2w.transpose();
	ocam_world(0) = r_sim_2d(0);	ocam_world(1) = r_sim_2d(1);	ocam_world(2) = 0;
}

Vector2f nbv_controller::getNBVinRF(){
	// Rotate target pk and Uk to relative frame
	if (sim) {
	} else {
		ros::spinOnce();
		r_sim_2d(0) = ocam_world(0);	r_sim_2d(1) = ocam_world(1);
		R_sim_2d(0,0) = Rot_c2w(0,0);
		R_sim_2d(0,1) = Rot_c2w(0,2);
		R_sim_2d(1,0) = Rot_c2w(1,0);
		R_sim_2d(1,1) = Rot_c2w(1,2);
	}
	
//	Vector3f p3d = objective.pk;
	Matrix3f V3d = Rot_w2c * objective.Uk * Rot_w2c.transpose();

	Vector2f p_star = getRelativePosition(R_sim_2d, r_sim_2d, objective.xk);
//	p_star(0) = p3d(0);
//	p_star(1) = p3d(2);
	Matrix2f V = matrix2d(1,3,V3d);

//	// Check input
//	cout << "p_star: " << p_star.transpose() << endl << endl;
//	cout << "V: " << endl << V << endl << endl;

	if (show_iter) {
		cout << "Initial in relative control frame: p_star = [ " << p_star.transpose() << " ]... " << endl << endl; 
	}
	p_star = relUpdate(p_star, V);
	if (show_iter) {
		cout << "Update in relative control frame: p_star = [ " << p_star.transpose() << " ]... " << endl << endl;
	}	
	return p_star;
}

Vector2f nbv_controller::relUpdate(Vector2f p_star, Matrix2f V){
	// Save old relative vector
	Vector2f p_old = p_star;

	// Initialize iteration of gradient descent
	int rel_iter = 1;
	bool indicate = false;
	Vector2f p_star_new, dist;
	float val_old;
	float rel_backtrack = 0;

	// Start iteration
	if (show_iter) {
		cout << "Start relative coordinates gradient descent." << endl << endl;
	}
	while (!indicate){
		// Evaluate function value and gradient direction
		evalFuncGrad(p_star, V, true);
		val_old = val;

//		// Check val and dh
//		cout << "val: " << val << endl << endl;
//		cout << "dh: " << dh.transpose() << endl << endl;
//		indicate = true;

		// Try going the full delta in the gradient direction
		p_star_new = p_star - delta * relgains * dh / dh.norm();

		// Evaluate function again
		evalFuncGrad(p_star_new, V, false);

		rel_backtrack = 0;
		while( val > val_old){
			if (rel_backtrack == 0) {
				if (show_iter) {
					cout << "Doing backtrack in gradient descent in relative frame" << endl << endl;
				}
			}
			p_star_new = p_star - pow(0.9, rel_backtrack) * delta * dh / dh.norm();
			evalFuncGrad(p_star_new, V, false);
			if (rel_backtrack > 20){
				if (show_iter) {
					cout << "Relative update couldn't improve." << endl << endl;
				}
				indicate = true;
				break;
			}
			rel_backtrack++;
		}
		p_star = p_star_new;
		rel_iter++;
		dist = p_old - p_star;
		if (dist.norm() > delta - tol){
			p_star = p_old + delta * (p_star - p_old) / dist.norm();
			indicate = true;
		}
		if (rel_iter >= it_max){
			indicate = true;
			if (show_iter) {
				cout << "Relative update timed out!" << endl << endl;
			}
		}
	}
	return p_star;
}

void nbv_controller::evalFuncGrad(Vector2f p, Matrix2f V, bool eval_dh){
	Vector2f pixels = measure_pixel(p);
	Matrix2f mCov = observeCov(pixels);
	Matrix2f temp = V.inverse() + mCov.inverse();
	Matrix2f Xi = temp.inverse();

//	// Check result
//	cout << "pixels: " << pixels.transpose() << endl << endl;
//	cout << "mCov: " << endl << mCov << endl << endl;
//	cout << "Xi: " << endl << Xi << endl << endl;

	// Trace or Frobenius norm of new covariance matrix
	val = Xi.trace();
//	val = Xi.norm();

	if (eval_dh){
		// Calcuate dh
		dh = calc_DJQJ_p(p, mCov, Xi, pixels);
	}	
}

Eigen::Vector2f nbv_controller::measure_pixel(Eigen::Vector2f p){
	Vector2f pixels;
	pixels[0] = flength * (p[0] + 0.5) / p[1];
	pixels[1] = flength * (p[0] - 0.5) / p[1];
//	pixels[2] = flength * p[1] / p[2];
	return pixels;
}

Eigen::Matrix2f nbv_controller::observeCov(Eigen::Vector2f pixels){
	Matrix2f J, U;
	float xl, xr, y, d, invSqd;
	xl = pixels[0];	xr = pixels[1];
	d = xl - xr;
	invSqd = 1 / pow(d, 2);

	float dpx_dxl, dpx_dxr, dpz_dxl, dpz_dxr;
	dpx_dxl = invSqd * (-xr);
	dpx_dxr = invSqd * xl;
	dpz_dxl = invSqd * (-flength);
	dpz_dxr = invSqd * (flength);
	J << dpx_dxl, dpx_dxr, dpz_dxl, dpz_dxr;
	U = J * Q * J.transpose();
	return U;
}

Eigen::Vector2f nbv_controller::calc_DJQJ_p(Eigen::Vector2f p, Eigen::Matrix2f mCov, Eigen::Matrix2f Xi, Vector2f pixels){
	Matrix2f M1 = mCov.inverse() * Xi * Xi * mCov.inverse();
	
	// Calculate derivative of xl, xr, y on px, py, pz;
	float dxl_dpx, dxl_dpz, dxr_dpx, dxr_dpz;
	dxl_dpx = flength / p[1];
	dxl_dpz = -flength * (p[0] + bline/2) / pow(p[1], 2);
	dxr_dpx = flength / p[1];
	dxr_dpz = -flength * (p[0] - bline/2) / pow(p[1], 2);

	// Calculate derivative of JQJ' on xl, xr, y;
	float xl, xr, y, d, bonsqrd;
	xl = pixels[0]; xr = pixels[1]; d = xl - xr; bonsqrd = bline / pow(d, 2);

	Matrix2f M, J, dJ_dxl, dJ_dxr, dJ_dy, dJQJ_xl, dJQJ_xr, dJQJ_y, temp;

	M << -xr, xl, -flength, flength;
	J = bonsqrd * M;

	temp << 0, bonsqrd, 0, 0;
	dJ_dxl = temp - bonsqrd / d * M;

	temp << -bonsqrd, 0, 0, 0;
	dJ_dxr = temp + bonsqrd / d * M;

	dJQJ_xl = dJ_dxl * Q * J.transpose() + J * Q * dJ_dxl.transpose();
	dJQJ_xr = dJ_dxr * Q * J.transpose() + J * Q * dJ_dxr.transpose();

	// Calculate derivative of JQJ' on px, py, pz;
	Matrix2f dJQJ_px, dJQJ_pz;
	dJQJ_px = dJQJ_xl * dxl_dpx + dJQJ_xr * dxr_dpx;
	dJQJ_pz = dJQJ_xl * dxl_dpz + dJQJ_xr * dxr_dpz;

	// Calculate derivative of h on px, py, pz;
	Vector2f dh1;
	float dh_px, dh_pz;
	temp = M1 * dJQJ_px;	dh_px = temp.trace();
	temp = M1 * dJQJ_pz;	dh_pz = temp.trace();
	dh1 << dh_px, dh_pz;
	
	return dh1;
}

// *********************** NBV in Global control frame *********************** //
void nbv_controller::getNBVinGF(Vector2f p_star){
//	ros::spinOnce();
	glob_setup(p_star);

	glblUpdate();
}

void nbv_controller::glob_setup(Vector2f p_star){
	Vector2f p_old, r, temp;
//	Vector3f p3d = objective.pk;
//	p_old(0) = p3d(0);
//	p_old(1) = p3d(2);
	p_old = getRelativePosition(R_sim_2d, r_sim_2d, objective.xk);
	Matrix2f R;
	if (sim) {
		r = r_sim_2d;
		R = R_sim_2d;
	} else {
		ros::spinOnce();
		r(0) = ocam_world(0);
		r(1) = ocam_world(1);
		R << Rot_c2w(0,0), Rot_c2w(0,2),
		     Rot_c2w(1,0), Rot_c2w(1,2);
	}

	goal_r = r + R * (p_old - p_star);

	Vector2f zstar;
	zstar(0) = objective.xk(0);	zstar(1) = objective.xk(1);
	temp = zstar - goal_r;
	newDir = temp / temp.norm();
	oldDir << 0, 1;

//	cout << "goal r is : " << goal_r.transpose() << endl << endl;
//	cout << "current R is : " << endl << R << endl << endl;
//	cout << "new Dir: " << newDir.transpose() << endl << endl;
//	cout << "old Dir: " << oldDir.transpose() << endl << endl;
}

void nbv_controller::glblUpdate(){
	Vector2f r, r_old, temp;
	Matrix2f R;
	
	if (sim) {
		r = r_sim_2d;
		R = R_sim_2d;
	} else {
		ros::spinOnce();
		r(0) = ocam_world(0);
		r(1) = ocam_world(1);
		R << Rot_c2w(0,0), Rot_c2w(0,2),
		     Rot_c2w(1,0), Rot_c2w(1,2);
	}
	r_old = r;
	
	bool indicate = false;
	int glob_iter = 0;
	float val_old;
	while(!indicate) {

// ************************ Yan's version ************************ //
//		glbl(r, R, true);
//		val_old = val_glob;

////		// Test value evaluation
////		cout << "val_old: " << val_old << endl << endl;
////		indicate = true;

////		// Test derivative of phi on r
////		cout << "dphix_dr: " << dphix_dr.transpose() << endl << endl;
////		cout << "dphiz_dr: " << dphiz_dr.transpose() << endl << endl;
////		indicate = true;

////		//Test r_inv, R_inv;
////		cout << "R_upd: " << endl << R_upd << endl << endl;
//		
//		glbl(r_upd, R_upd, false);

////		cout << "r_old : " << r_old << endl << endl;
////		cout << "R_old : " << R << endl << endl;
////		cout << "r_upd : " << r_upd << endl << endl;
////		cout << "R_upd : " << R_upd << endl << endl;
////		cout << "val_old : " << val_old << endl << endl;
////		cout << "val_new : " << val_glob << endl << endl;
////		
//		// Check if gradient descent decrease the value of objective function
//		if (val_glob < val_old){
//			r = r_upd;
//			R = R_upd;
//		} else {
//			cout << "Gradient descent increase the value of objective function at iteration " << glob_iter << " !" << endl << endl;
//			indicate = true;
//		}

// ********************* Charlie's version ************************ //
		glbl(r, R, true);

////		// Test derviative gradx, gradz, grad, dpsihat_dr
//		cout << "gradx: " << gradx.transpose() << endl << endl;
//		cout << "gradz: " << gradz.transpose() << endl << endl;
//		cout << "dpsihat_dr: " << dpsihat_dr.transpose() << endl << endl;

//		waitForEnter();

		if (glob_iter > 1) {
			if (val_glob > val_old) {
				if (show_iter) {
					cout << "Gradient descent increase the value of objective function at iteration " << glob_iter << " !" << endl << endl;
				}
				indicate = true; 
			}
		}

		r = r_upd;
		R = R_upd;
		val_old = val_glob;

		glob_iter++;

		temp = r - r_old;
		if (temp.norm() > delta){
			if (show_iter) {
				cout << "Global update travelled max distance of delta(" << delta << ")." << endl << endl;
			}
			indicate = true;
		} else if (val_glob < tol) {
			if (show_iter) {
				cout << "Global objective function descents to zero..." << endl << endl;
			}
			indicate = true;
		} else if (glob_iter > it_max){
			if (show_iter) {
				cout << "Global update timed out!" << endl << endl;
			}
			indicate = true;
		}
	}

	r_new = r;
	R_new = R;

//	cout << "new r is : " << r.transpose() << endl << endl;
//	cout << "new R is : " << endl << R << endl << endl;
}

void nbv_controller::glbl(Vector2f r, Matrix2f R, bool eval_grad){
	float objective_only, penalty_contribution;
	// Compute the objective function value
	posegoal(r, goal_r, eval_grad);
	rotgoal(R, newDir, oldDir, eval_grad);
	objective_only = gTrade * val_obj_r + (1-gTrade) * val_obj_R;

	// Test objective value
//	cout << "objective_only: " << objective_only << endl << endl;

	// Compute the penalty and barrier function values
	penfunc(r, R, eval_grad);
	barr(eval_grad);
	penalty_contribution = pen / ((float)num_targets) * pen_phi.sum();

	// Compute their weighted sum
	val_glob = objective_only + penalty_contribution;

	// Compute the gradient and new r and R if requested
	if (eval_grad){
		// Compute new r
		phi_chain2global(R);

		calc_grad_fov_r();

		dpsihat_dr = dpsi_dr + pen / ((float)num_targets) * grad;
		r_upd = r - gTrade * Dt * dpsihat_dr;
		
//		// Compute new R using 4th order Runge Kutta Update to rotation
//		// Test input to gradflow
//		cout << "Input to gradflow r: " << r.transpose() << endl << endl;
//		cout << "Input to gradflow R: " << endl << R << endl << endl;
		k1 = gradflow(r, R);
		k2 = gradflow(r, R + h / 2 * k1);
		k3 = gradflow(r, R + h / 2 * k2);
		k4 = gradflow(r, R + h * k3);	
		R_upd = R - (1 - gTrade) * (h/6) * (k1 + 2 * k2 + 2 * k3 + k4);

		// Test k1-k4
//		cout << "k1: " << endl << k1 << endl << endl;
//		cout << "k2: " << endl << k2 << endl << endl;
//		cout << "k3: " << endl << k3 << endl << endl;
//		cout << "k4: " << endl << k4 << endl << endl;

		// Project R to closest SO3 matrix
		if (R_upd.determinant() > 1 + sqrt(std::numeric_limits<float>::epsilon()) || R_upd.determinant() < 1 - sqrt(std::numeric_limits<float>::epsilon())){
			JacobiSVD<MatrixXf> svd(R_upd, ComputeFullU | ComputeFullV);
			R_upd = svd.matrixU() * svd.matrixV().transpose();
//			cout << "U: " << endl << svd.matrixU() << endl << endl;
//			cout << "V: " << endl << svd.matrixV() << endl << endl;
		}
	}
}

// Global control frame: calculate derivative on R
Matrix2f nbv_controller::gradflow(Vector2f r, Matrix2f R){
	calc_dpsihat_dR(r, R);
	return -R * dpsihat_dR;
}

void nbv_controller::calc_dpsihat_dR(Vector2f r, Matrix2f R){
	penGrad = Matrix2f::Zero();
	// Compute derivative of px, py, pz on R
	for (int ii = 0; ii < num_targets; ii++){
		dpx_dR[ii] = 0.5 * (R.transpose() * (zi[ii] - r) * e1.transpose() - e1 * (zi[ii] - r).transpose() * R);
		dpz_dR[ii] = 0.5 * (R.transpose() * (zi[ii] - r) * e2.transpose() - e2 * (zi[ii] - r).transpose() * R);
//		// Test dpx/z_dR
//		cout << "dpx_dR: " << endl << dpx_dR[ii] << endl << endl;
//		cout << "dpz_dR: " << endl << dpz_dR[ii] << endl << endl;
	}

	// Compute derivate of phix, phiy, phiz on R
	for (int ii = 0; ii < num_targets; ii++){
		dphix_dR[ii] = dphix_dpx(0, ii) * dpx_dR[ii] + dphix_dpz(0, ii) * dpz_dR[ii];
		dphiz_dR[ii] = dphiz_dpx(0, ii) * dpx_dR[ii] + dphiz_dpz(0, ii) * dpz_dR[ii];
//		// Test dphix/z_dR
//		cout << "dphix_dR: " << endl << dphix_dR[ii] << endl << endl;
//		cout << "dphiz_dR: " << endl << dphiz_dR[ii] << endl << endl;
	}

	// Compute the penalty function gradients
	for (int ii = 0; ii < num_targets; ii++){
		penGrad = penGrad + dpen_phi(0, ii) * dphix_dR[ii] + dpen_phi(1, ii) * dphiz_dR[ii];
	}

//	// Test penGrad
//	cout << "penGrad: " << endl << penGrad << endl << endl;

	if (pen / ((float)num_targets) * penGrad.lpNorm<Infinity>() < 0.5 * dpsi_dR.lpNorm<Infinity>()){
		penGrad = Matrix2f::Zero();
	}

	if (pen / ((float)num_targets) * penGrad.lpNorm<Infinity>() > dpsi_dR.lpNorm<Infinity>()){
		if (show_iter) {
			cout << "FoV gradient on R is huge compared to objective of realizing NBV!" << endl << endl;
		}
	}

	dpsihat_dR = dpsi_dR - pen / ((float)num_targets) * penGrad;

//	// Test dpsihat_dR
//	cout << "dpsihat_dR: " << endl << dpsihat_dR << endl << endl;

	if (dpsihat_dR.lpNorm<Infinity>() < std::numeric_limits<float>::epsilon()){
		if (show_iter) {
			cout << "Gradient of Rotation matrix is nearly zero!" << endl << endl;
		}
		dpsihat_dR = Matrix2f::Zero();
	}

	if (checkSkewSymm(dpsihat_dR)){
		dpsihat_dR(0, 1) = -dpsihat_dR(1, 0);
	}

	if (dpsihat_dR.diagonal().norm() > tol){
		dpsihat_dR(0, 0) = 0;
		dpsihat_dR(1, 1) = 0;
		if (show_iter) {
			cout << "Gradient of rotation has some nonzero diagonals" << endl << endl;
		}
	}
}

bool nbv_controller::checkSkewSymm(Matrix2f M){
	float temp = fabs(M(0,1) + M(1,0));
	if (temp > std::numeric_limits<float>::epsilon()){
		if (show_iter) {
			cout << "Gradient of rotation is not skew symmetric!" << endl << endl;
		}
		return true;
	} else {
		return false;
	}
}

// Global control frame: calculate derivative on r
void nbv_controller::posegoal(Vector2f r, Vector2f goal_r, bool eval_grad){
	Vector2f temp;
	temp = goal_r - r;
	val_obj_r = pow(temp.norm(), 2);
	// Compute the derivative of objective on r
	if (eval_grad){
		dpsi_dr = 2 * (r - goal_r);
////		// Test dpsi_dr
//		cout << "dpsi_dr: " << dpsi_dr.transpose() << endl << endl;
	}
}

void nbv_controller::rotgoal(Matrix2f R, Vector2f y, Vector2f n, bool eval_grad){
	Vector2f temp;
	temp = R.transpose() * y - n;
	val_obj_R = pow(temp.norm(), 2);
	// Compute the derivative of objective on R
	if (eval_grad){
		dpsi_dR = temp * y.transpose() * R - R.transpose() * y * temp.transpose();
//		// Test dpsi_dR
//		cout << "dpsi_dR: " << endl << dpsi_dR << endl << endl;
	}
}

void nbv_controller::penfunc(Vector2f r, Matrix2f R, bool eval_grad){
	MatrixXf temp;
	temp.resize(1, num_targets); temp.setZero();

	for (int ii = 0; ii < num_targets; ii++){
		p.col(ii) = getRelativePosition(R, r, targets[ii]->ftr_meas_history[count_meas]);
	}

//	cout << "p: " << endl << p << endl << endl;

	temp = (wi * p.row(1)).array() - bline * flength;
	phi.row(0) = temp.array() * temp.array() / (4 * pow(flength, 2)) - p.row(0).array() * p.row(0).array();
	phi.row(1) = p.row(1).array() * p.row(1).array() - pow(bline * flength / wi, 2);

	// Test phi;
//	cout << "phi: " << phi.transpose() << endl << endl;

	if (eval_grad){
		dphix_dpx = -2 * p.row(0);
		dphix_dpz = 2 * (((wi * p.row(1)).array() - bline * flength) / (2 * flength)) * (wi / (2 * flength));
		dphiz_dpx.setZero();
		dphiz_dpz = 2 * p.row(1);
//		// Test derivative of phix,z on px,z;
//		cout << "dphix_dpx: " << dphix_dpx << endl << endl;
//		cout << "dphix_dpz: " << dphix_dpz << endl << endl;
//		cout << "dphiz_dpx: " << dphiz_dpx << endl << endl;
//		cout << "dphiz_dpz: " << dphiz_dpz << endl << endl;
	}
}

void nbv_controller::barr(bool eval_grad){
	pen_phi = 1 / phi.array();
	if (eval_grad){
		dpen_phi = -1 / (phi.array() * phi.array());
//		// Test derivative of penalty function of phi
//		cout << "dpen_phi: " << dpen_phi.transpose() << endl << endl;
	}
}

void nbv_controller::phi_chain2global(Matrix2f R){
	Vector2f dp_drx, dp_dry, dp_drz;
	// Compute derivative of px, py, pz w.r.t. rx, ry, rz in global coordinates
	dp_drx = -R.transpose() * e1;
	dp_drz = -R.transpose() * e2;
	// Compute derivative of phix, phiy, phiz w.r.t. rx, ry, rz in global coordinates
	dphix_dr.row(0) = dphix_dpx * dp_drx[0] + dphix_dpz * dp_drx[1];
	dphix_dr.row(1) = dphix_dpx * dp_drz[0] + dphix_dpz * dp_drz[1];

	dphiz_dr.row(0) = dphiz_dpx * dp_drx[0] + dphiz_dpz * dp_drx[1];
	dphiz_dr.row(1) = dphiz_dpx * dp_drz[0] + dphiz_dpz * dp_drz[1];
}

void nbv_controller::calc_grad_fov_r(){
	gradx[0] = (dpen_phi.row(0).array() * dphix_dr.row(0).array()).matrix().sum();
	gradx[1] = (dpen_phi.row(0).array() * dphix_dr.row(1).array()).matrix().sum();

	gradz[0] = (dpen_phi.row(1).array() * dphiz_dr.row(0).array()).matrix().sum();
	gradz[1] = (dpen_phi.row(1).array() * dphiz_dr.row(1).array()).matrix().sum();
	
	grad = gradx + gradz;

	if (pen / ((float)num_targets) * grad.norm() < 0.5 * dpsi_dr.norm()){
		grad = Vector2f::Zero();
	}

	if (pen / ((float)num_targets) * grad.norm() > 2.0 * dpsi_dr.norm()){
		if (show_iter) {
			cout << "FoV gradient on r is huge compared to objective of realizing NBV!" << endl << endl;
		}
	}
}

Matrix2f nbv_controller::matrix2d(int index1, int index2, Matrix3f mat){
	Matrix2f mat_new;
	mat_new(0, 0) = mat(index1-1, index1-1);
	mat_new(0, 1) = mat(index1-1, index2-1);
	mat_new(1, 0) = mat(index2-1, index1-1);
	mat_new(1, 1) = mat(index2-1, index2-1);
	return mat_new;
}

void nbv_controller::waitForEnter(){
	char enter;
	cout << "Type any key to continue:";
	cin >> enter;
	cin.clear();
	cin.sync();
}

void nbv_controller::calcOrder(){
	ros::spinOnce();
	Vector3f cam_goal, hand_goal;
	cam_goal(0) = r_new(0);	cam_goal(1) = r_new(1);	cam_goal(2) = ocam_world(2);
	hand_goal = cam_goal - Rot_h2w * ocam_hand;
	r_hand(0) = hand_goal(0); r_hand(1) = hand_goal(1);

	Matrix3f Rcam_goal, Rhand_goal;
	Rcam_goal << R_new(0,0), 0, R_new(0,1),
		     R_new(1,0), 0, R_new(1,1),
		     0,		-1, 0;
	Rhand_goal = Rcam_goal * Rot_c2h.transpose();
	R_hand = matrix2d(1, 2, Rhand_goal);
}

#endif
