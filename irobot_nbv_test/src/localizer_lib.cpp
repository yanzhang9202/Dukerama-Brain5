#ifndef ___LOCALIZER_LIB_CPP___
#define ___LOCALIZER_LIB_CPP___

#include "localizer_lib.h"

using namespace std;
using namespace Eigen;

// ******************** Public ********************** //

nbv_localizer::nbv_localizer(ros::NodeHandle &nh, int ds, int num_t, bool simu):
outfile_meas("/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/Results/meas.txt"),
outfile_measCov("/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/Results/measCov.txt"),
outfile_ftrm("/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/Results/ftrm.txt"),
outfile_ftrCov("/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/Results/ftrCov.txt") {
	downsample = (float)ds;
	cout << "Initialize localizer..." << endl;

	vrpn_sub = nh.subscribe<geometry_msgs::TransformStamped>("Brain5Camera/pose", 1, &nbv_localizer::vrpnCallback, this);
	num_targets = num_t;
	if (num_targets < 1){ 
		cout << "Error: No targets' info !" << endl;
	} else {
		targets.reserve(num_targets);
		ctr_left.reserve(num_targets); ctr_right.reserve(num_targets);
	}
	count_meas = 0;

	for (int i = 0; i < num_targets; i++){
		targets.push_back(new target());
	}

	sim = simu;
	if (sim) {
		bline = 1.0;
		wi = 1024.0;
		fov = 70 * (PI / 180);
		alphaq = tan(fov / 2);
		c = 1.0/(2 * alphaq);
		flength = (0.5 * wi) / alphaq;
	}
	if (sim) {
		cout << "flength: " << flength << endl;
	}

	if (simu) {
		getCameraUncertainty();
	} else {
		getCameraCalib();
		getCameraUncertainty();
	}

	q_h = Vector4f::Zero();
	ohand_world = Vector3f::Zero();
}

void nbv_localizer::test_vrpn(){
	char enter;
	cout << "Type in any key to get data from VRPN:";
	cin >> enter;
	ros::spinOnce();
	cout << "Response from VRPN: " << endl;
//	cout << ocam_world.transpose() << endl;
	cout << q_h.transpose() << endl;
	cout << ohand_world.transpose() << endl;
	cin.clear();
	cin.sync();
}

void nbv_localizer::calcMeasurements(std::vector<Eigen::Vector2f> target_ctr_left, std::vector<Eigen::Vector2f> target_ctr_right, std::vector<bool> flag){
	ctr_left = target_ctr_left;	ctr_right = target_ctr_right;
	ros::spinOnce();	// Update camera pose information
	for (int i = 0;  i < num_targets; i++){
		if (flag[i]){
			cout << "target " << i+1 << " 's info is being filtered!" << endl;
			xl = ctr_left[i][0] - cc_left[0];
			xr = ctr_right[i][0] - cc_right[0];
			y = (ctr_left[i][1] - cc_left[1] + ctr_right[i][1] - cc_right[1])/2;
			targets[i]->meas_history.push_back(triangulation(xl, xr, y));
			targets[i]->measCov_history.push_back(calc_sigma(xl, xr, y));
			// Kalman Filter
			targets[i]->KalmanFilter(count_meas);
//			targets[i]->ftr_pk_history.push_back(getRelativePosition(targets[i]->ftr_meas_history[count_meas]));
		} else {
			cout << "target " << i+1 << " is missing!" << endl;
			if (count_meas == 0){
				targets[i]->meas_history.push_back(Vector3f::Zero());
				targets[i]->measCov_history.push_back(100*Matrix3f::Identity());
				targets[i]->ftr_meas_history.push_back(Vector3f::Zero());
				targets[i]->ftr_measCov_history.push_back(100*Matrix3f::Identity());
			} else {
				targets[i]->meas_history.push_back(targets[i]->meas_history[count_meas-1]);
				targets[i]->measCov_history.push_back(targets[i]->measCov_history[count_meas-1]);
				targets[i]->ftr_meas_history.push_back(targets[i]->ftr_meas_history[count_meas-1]);
				targets[i]->ftr_measCov_history.push_back(targets[i]->ftr_measCov_history[count_meas-1]);
			}
		}
		writeMatrix(outfile_meas, targets[i]->meas_history[count_meas]);
		writeMatrix(outfile_measCov, targets[i]->measCov_history[count_meas]);
		writeMatrix(outfile_ftrm, targets[i]->ftr_meas_history[count_meas]);
		writeMatrix(outfile_ftrCov, targets[i]->ftr_measCov_history[count_meas]);
	}

	// Show single measurement results;
	cout << "The " << count_meas+1 << " th measurements: " << endl << endl; 
	showFtrMeasurements(count_meas);
//	cout << "The covariance matrix: " << endl;
//	cout << calc_sigma(xl, xr, y);

	count_meas++;
	if (count_meas == NUM_MEAS){
		outfile_meas.close();
		outfile_measCov.close();
		outfile_ftrm.close();
		outfile_ftrCov.close();
	}
}

void nbv_localizer::simMeasurements(std::vector<Eigen::Vector2f> target_ctr_left, std::vector<Eigen::Vector2f> target_ctr_right, Eigen::Vector2f r, Eigen::Matrix2f R){
	ctr_left = target_ctr_left;	ctr_right = target_ctr_right;
//	ros::spinOnce();	// Update camera pose information
	for (int i = 0;  i < num_targets; i++){
		xl = ctr_left[i][0];
		xr = ctr_right[i][0];
		y = (ctr_left[i][1] + ctr_right[i][1])/2;
		targets[i]->meas_history.push_back(simtriangulation(xl, xr, y, r, R));
		targets[i]->measCov_history.push_back(simcalc_sigma(xl, xr, y, r, R));
		// Kalman Filter
		targets[i]->KalmanFilter(count_meas);
//		targets[i]->ftr_pk_history.push_back(getRelativePosition(targets[i]->ftr_meas_history[count_meas]));	
	}

//	// Show single measurement results;
//	cout << "The " << count_meas+1 << " th measurements: " << endl << endl; 
//	showMeasurements(count_meas);
//	showFtrMeasurements(count_meas);
//	cout << "The covariance matrix: " << endl;
//	cout << calc_sigma(xl, xr, y);

	count_meas++;
}

// ********************* Private ********************* //
// ********************* Calculate single measurements and covariance matrix Sigma ********************** //
Vector3f nbv_localizer::triangulation(float xl, float xr, float y){
	// Triangulation in relative camera frame;
	Vector3f pk, xk;
  	pk(0)=(bline * (xl + xr))/(2 * (xl - xr));
  	pk(1)=(bline * y)/(xl - xr);
  	pk(2)=(bline * flength)/(xl - xr);
//	cout << "pk: " << pk.transpose() << endl << endl;
	// Transform to global coordinates;
	xk = Rot_c2w * pk + ocam_world;
	return xk;
}

Vector3f nbv_localizer::simtriangulation(float xl, float xr, float y, Vector2f r, Matrix2f R){
//	// Test xl, xr, y, r R
//	cout << "xl: " << xl << endl;
//	cout << "xr: " << xr << endl;
//	cout << "r: " << r.transpose() << endl;
//	cout << "R: " << endl << R << endl << endl;
	
	// Triangulation in relative camera frame;
	Vector3f pk, xk;
  	pk(0)=(bline * (xl + xr))/(2 * (xl - xr));
  	pk(1)=(bline * y)/(xl - xr);
  	pk(2)=(bline * flength)/(xl - xr);
	// Transform to global coordinates;
	Vector2f pk2d, xk2d;
	pk2d(0) = pk(0);
	pk2d(1) = pk(2);
	xk2d = R * pk2d + r;
	xk(0) = xk2d(0);	xk(1) = xk2d(1);	xk(2) = pk(1);

//	// Test
//	cout << "pk2d: " << pk2d.transpose() << endl << endl;

	return xk;
}

Matrix3f nbv_localizer::calc_sigma(float xl, float xr, float y){
  Matrix3f J = calc_J(xl,xr,y);
  Matrix3f sigma_meas = Rot_c2w * J * Q * J.transpose() * Rot_c2w.transpose();
//  std::cout << "Rotation determinant is: " << Rot_c2w.determinant() << std::endl << std::endl;
  return sigma_meas;
}

Matrix3f nbv_localizer::simcalc_sigma(float xl, float xr, float y, Vector2f r, Matrix2f R){
  Matrix3f J = calc_J(xl,xr,y);
  Matrix2f J_2d, Q_2d;
  Q_2d = matrix2d(1,2,Q);
  J_2d << J(0,0), J(0,1), J(2,0), J(2,1);
  Matrix2f sigma_meas_2d = R * J_2d * Q_2d * J_2d.transpose() * R.transpose();
  Matrix3f sigma_meas;
  sigma_meas << sigma_meas_2d(0, 0), sigma_meas_2d(0, 1), 0,
	        sigma_meas_2d(1, 0), sigma_meas_2d(1, 1), 0,
		0, 		     0, 	          1;
//  std::cout << "Rotation determinant is: " << Rot_c2w.determinant() << std::endl << std::endl;
  return sigma_meas;
}

Matrix2f nbv_localizer::matrix2d(int index1, int index2, Matrix3f mat){
	Matrix2f mat_new;
	mat_new(0, 0) = mat(index1-1, index1-1);
	mat_new(0, 1) = mat(index1-1, index2-1);
	mat_new(1, 0) = mat(index2-1, index1-1);
	mat_new(1, 1) = mat(index2-1, index2-1);
	return mat_new;
}

Matrix3f nbv_localizer::calc_J(float xl, float xr, float y){
  float disp = xl - xr;
  Matrix3f J;
  J << -bline*xr, bline*xl, 0, -bline*y, bline*y, bline*disp, -bline*flength, bline*flength, 0;
  J=J/(disp*disp);
  return J;
}

void nbv_localizer::showMeasurements(int index){
	for (int i = 0; i < num_targets; i++){
		cout << "Target " << i+1 << ": " << targets[i]->meas_history[index].transpose() << endl;
		if (sim) {
			cout << "Target " << i+1 << " single covaraince: " << endl << targets[i]->measCov_history[index] << endl;
		}
	}
	cout << endl;
}

void nbv_localizer::showFtrMeasurements(int index){
	for (int i = 0; i < num_targets; i++){
		cout << "Target " << i+1 << ": " << targets[i]->ftr_meas_history[index].transpose() << endl;
		if (sim) {
			cout << "Target " << i+1 << " filtered covaraince: " << endl << targets[i]->ftr_measCov_history[index] << endl;
		} else {
			cout << "Trace of target " << i+1 << ": " << targets[i]->ftr_measCov_history[index].trace() << endl;
		}
	}
	cout << endl;
}

void nbv_localizer::getCameraCalib(){
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

  	cout << "bline: " << bline << "(m)"<< endl << endl;
  	cout << "flength: " << flength << "(pixel)" << endl << endl;
  	cout << "cc_left: " << cc_left(0) << " " << cc_left(1) << endl << endl;
  	cout << "cc_right: " << cc_right(0) << " " << cc_right(1) << endl << endl;
  	cout << "Hc2m_left: " << "\n" << Hc2m_left << endl << endl;
  	cout << "Hc2m_right: " << "\n" << Hc2m_right << endl << endl;

  	for (line = 0; line < 3; line++){
      		ocam_hand(line) = (Hc2m_left(line, 3) + Hc2m_right(line, 3))/2/1000;
  	}

  	flength = flength/downsample;
  	cout << "flength: " << flength << " by down-sample rate " << downsample << endl << endl;
	cc_left = cc_left/downsample;	cc_right = cc_right/downsample;
  	cout << "cc_left: " << cc_left(0) << " " << cc_left(1) << " by down-sample rate " << downsample << endl << endl;
  	cout << "cc_right: " << cc_right(0) << " " << cc_right(1) << " by down-sample rate " << downsample << endl << endl;		
}

void nbv_localizer::getCameraUncertainty(){
	int line, col;
	ifstream file;
  	file.open("/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/Camera_Config/Q.txt");
    
  	if (!file.is_open()){
    		cout << "No 'Q' file in the /include/Camera_Config folder!\n";
    		return; 
  	}

	for (line = 0; line < 3; line++){
		for (col = 0; col < 3; col++){
			file >> Q(line, col);	
		}
	}
	file.close();

	cout << "Uncertainty matrix on image plane: " << endl;
	cout << Q << endl;
}

void nbv_localizer::vrpnCallback(const geometry_msgs::TransformStamped msg){
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

//    q_h[0] =msg->transform.rotation.x;
//    q_h[1] =msg->transform.rotation.y;
//    q_h[2] =msg->transform.rotation.z;
//    q_h[3] =msg->transform.rotation.w;

//    ohand_world[0] = msg->transform.translation.x;
//    ohand_world[1] = msg->transform.translation.y;
//    ohand_world[2] = msg->transform.translation.z;
}

void nbv_localizer::writeMatrix(std::ofstream& out, Eigen::MatrixXf matrix){
	for (int line = 0; line < matrix.rows(); line++){
		for (int col = 0; col < matrix.cols(); col++){
			if (line == (matrix.rows()-1) && col == (matrix.cols()-1)){
				out << matrix(line, col) << "\n";
			} else {
				out << matrix(line, col) << " ";
			}
		}
	}
}

#endif
