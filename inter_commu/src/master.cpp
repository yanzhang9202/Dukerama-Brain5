#ifndef ___MASTER_CPP___
#define ___MASTER_CPP___

#include "master.h"

using namespace std;
using namespace Eigen;

//	Public
ic_master::ic_master(ros::NodeHandle &nh):sub_rate(5){
	robotName.resize(num_robot, 1);
	show_sys_info = true;
	loadRobotInfo();
	peers_info.reserve(num_robot);

	my_sub.reserve(num_robot);
	for (int i = 0; i < num_robot; i++){
		stringstream ss;
		ss << "Brain" << (int)robotName(i, 0) << "/Msg";
		string sub_name =  ss.str();
		my_sub.push_back(nh.subscribe<std_msgs::Float32MultiArray>(sub_name.c_str(), 1, boost::bind(&ic_master::peerCallback, this, _1, i)));
	}
	string pub_name = "Master/Flag";
	my_pub = nh.advertise<std_msgs::Float32MultiArray>(pub_name.c_str(), 1);

	// Load Number
	loadInitialNum();

//	waitForEnter();
	state_flag = false;
}

void ic_master::supervise(){
	cout << "Start supervise!" << endl << endl;
	while(ros::ok() && (!state_flag)){
		publishmsg(0);
		ros::spinOnce();
		sub_rate.sleep();
	}
	cout << "Goal finished!" << endl << endl;
	double start_time = ros::Time::now().toSec();
	double duration = 0;
	while(ros::ok() && (duration < 10)){
		publishmsg(1);
		sub_rate.sleep();
		duration = ros::Time::now().toSec() - start_time;
	}
}

//	Private
// Load Info
void ic_master::loadRobotInfo(){
	string filename = "/home/dukerama/hydro_catkin/src/inter_commu/include/inter_commu/robotInfo.txt";
	readMatrix(filename, num_robot, 1, robotName);
	if (show_sys_info){
		cout << "Robot " << robotName.transpose() << " is involved in the experiment!" << endl << endl;
	}
}

void ic_master::loadInitialNum(){
	string filename = "/home/dukerama/hydro_catkin/src/inter_commu/include/inter_commu/initNum.txt";
	num_carry.resize(num_robot, 1);
	old_num.resize(num_robot, 1);
	error.resize(num_robot, 1);
	readMatrix(filename, num_robot, 1, num_carry);
	old_num = num_carry;
	goal = num_carry.mean();
	if (show_sys_info){
		cout << "Initial numbers are: " << endl << num_carry.transpose() << "." << endl << endl;
		cout << "Goal mean should be: " << goal << "." << endl << endl;
	}
}

void ic_master::readMatrix(string filename, int max_line, int max_col, MatrixXf &mat){
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

int ic_master::getNumberofLine(string filename){
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

void ic_master::peerCallback(const std_msgs::Float32MultiArray::ConstPtr& array, int i){
	int j = 0;	
	int decode = 0;
	to_who.clear();
	for (std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it){
		if (decode < 2){
			peers_info[i][j] = *it;
			j++;
		} else {
			to_who.push_back((int)(*it + 0.1));
		}
		decode++;
	}
	num_carry(i, 0) = peers_info[i][1];
	if ((num_carry - old_num).norm() > pow(0.1, 3)){
		error = num_carry.array() - goal;
		cout << "Update: current numbers: " << num_carry.transpose() << "." << endl;
		cout << "Update: current errors: " << error.transpose() << "." << endl << endl;
		old_num = num_carry;
		state_flag = checkError();
	}
}

bool ic_master::checkError(){
	bool temp = true;
	for (int i = 0; i < num_robot; i++){
		if (fabs(error(i, 0)) > tol){
			temp = false;
			break;
		}		
	}
	return temp;
}

void ic_master::publishmsg(float query){
	my_msg.data.clear();
	my_msg.data.push_back(query);
	my_pub.publish(my_msg);
}

#endif
