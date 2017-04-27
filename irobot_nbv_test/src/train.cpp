#ifndef ____TRAIN_CPP___
#define ____TRAIN_CPP___

#include "train.h"

using namespace std;

void waitForEnter(){
	cout << "Type any key to continue: ";
	char enter;
	cin >> enter;
	cin.clear();
	cin.sync();
}

void loadTarget(){
	string filename = "/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/Waypoints/InitTargets.txt";
	MatrixXf mat;
	mat.resize(1, 3);
	readMatrix(filename, 1, 3, mat);
	Targets = mat;
	cout << "target: " << endl << Targets << endl;
}

void loadWaypoint(){
	string filename = "/home/dukerama/hydro_catkin/src/irobot_nbv_test/include/Waypoints/Waypoints.txt";
	MatrixXf mat;
	int num_line = getNumberofLine(filename);
	mat.resize(num_line, 3);
	readMatrix(filename, num_line, 3, mat);
	Waypoint = mat;
	cout << "waypoint: " << endl << Waypoint.row(10) << endl;
}

void readMatrix(string filename, int max_line, int max_col, MatrixXf &mat){
	ifstream file;
	file.open(filename.c_str());
	if (!file.is_open()){
		cout << "File " << filename.c_str() << " is not found!" << endl << endl;
	}
	int num_line = getNumberofLine(filename);
	if (num_line < max_line){
		cout << "Number of lines in the file :" << filename.c_str() << " is not enough!" << endl << endl;
		exit(EXIT_FAILURE);
	}
	for (int line = 0; line < max_line; line++){
		for (int col = 0; col < max_col; col++){
			file >> mat(line, col);	
		}	
	}
	file.close();
}

int getNumberofLine(string filename){
	int number_of_lines = 0;
	ifstream file;
	string line;

	file.open(filename.c_str());
	    
	if (file.is_open()){
		while(std::getline(file, line)){
		      ++number_of_lines;
		}
	    	return number_of_lines;
	} else{ return 0; }
}

void vrpnCallback(const geometry_msgs::TransformStamped msg){
// Should add a condition that tgt_world data is valid;
  if (1)
//  if (wpt_request==0 && wpt_done==0)
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

void calcOrder(){
	cam_goal(0) = Waypoint(tst_count, 0);	cam_goal(1) = Waypoint(tst_count, 1);	cam_goal(2) = Waypoint(tst_count, 2);
	tgt = Targets.transpose();
	ros::spinOnce();
	hand_goal = cam_goal - Rot_h2w * ocam_hand;
	w_hat(0) = hand_goal(0); w_hat(1) = hand_goal(1);

	Eigen::Vector3f cam2targ;
	cam2targ = tgt - cam_goal;
	Rcam_goal = buildCamGoal(cam2targ/cam2targ.norm());
	Rhand_goal = Rcam_goal * Rot_c2h.transpose();
	Rg_hat = matrix2d(1, 2, Rhand_goal);

//	std::cout << "cam2targ: " << cam2targ.transpose() << endl;
//	std::cout << "Rcam_goal: " << endl << Rcam_goal << endl;
//	std::cout << "Rhand_goal: " << endl << Rhand_goal << endl;
	std::cout << "Rg_hat: " << endl << Rg_hat << endl;
}

Eigen::Matrix3f buildCamGoal(Eigen::Vector3f az){
	Eigen::Matrix3f mat;
	Eigen::Vector3f ax, ay;
	ax(0) = az(1); ax(1) = -az(0); ax(2) = 0;
	ay << 0, 0, -1;
	mat.col(0) = ax;
	mat.col(1) = ay;
	mat.col(2) = az;
	return mat;
}

Eigen::Matrix2f matrix2d(int index1, int index2, Eigen::Matrix3f mat){
	Eigen::Matrix2f mat_new;
	mat_new(0, 0) = mat(index1-1, index1-1);
	mat_new(0, 1) = mat(index1-1, index2-1);
	mat_new(1, 0) = mat(index2-1, index1-1);
	mat_new(1, 1) = mat(index2-1, index2-1);
	return mat_new;
}

void getCameraCalib(){
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
}

#endif
