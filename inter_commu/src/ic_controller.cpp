#ifndef ___IC_CONTROLLER_CPP___
#define ___IC_CONTROLLER_CPP___

#include "ic_controller.h"

using namespace Eigen;
using namespace std;

/*******	 Public		*******/

ic_controller::ic_controller(ros::NodeHandle &nh):pub_rate(2), outfile_number("/home/dukerama/hydro_catkin/src/inter_commu/include/inter_commu/number_record.txt"){
	sim = false;
	myName = 5;
	robotName.resize(num_robot, 1);
	show_sys_info = true;
	loadRobotInfo();
	findmyIndex();

	vrpn.reserve(num_robot);
	pose.reserve(num_robot);
	for (int i = 0; i < num_robot; i++){
		stringstream ss;
		ss << "Brain" << (int)robotName(i, 0) << "Camera/pose";
		string sub_name =  ss.str();
		vrpn.push_back(nh.subscribe<geometry_msgs::TransformStamped>(sub_name.c_str(), 1, boost::bind(&ic_controller::vrpnCallback, this, _1, i)));
	}

	// Load nodes info
	loadNodesInfo();

	waitForEnter();

	// Find my edge(r_ij) and my neighbours
	findEdges();
	findmyNeighbour();

	// Set my publisher and my subsriber to neighbour channels;
	setPub(nh);
	setSub(nh);

	to_who.reserve(num_robot);
	// Load policy
	loadPolicy();

	selfrecord.reserve(num_robot);
	peerrecord.reserve(num_robot);
	data_record.resize(num_robot);
	setrecord();

	// Load Number
	loadInitialNum();

	master_sub = nh.subscribe<std_msgs::Float32MultiArray>("Master/Flag", 1, &ic_controller::masterCallback, this);
	state_flag = false;

//	waitForEnter();
}

void ic_controller::test_vrpn(){
	waitForEnter();
	ros::spinOnce();
	for (int i = 0; i < num_robot; i++){
		cout << "Brain" << robotName(i,0) << " pose: [" << pose[i].transpose() << "]." << endl; 
	}
	cout << endl;
}

void ic_controller::waitForEnter(){
	char enter;
	cout << "Type into any key to continue: ";
	cin >> enter;
	cin.clear();
	cin.sync();
}

// Main process
void ic_controller::setInitialPose(){
	start_commu = false;
	int which_node = (int)policy(0, 2)-1;
	cout << "Brain " << myName << " sets initial position: node " << policy(0, 2) << " [" << nodes(0, which_node) << ", " << nodes(1, which_node) << "]." << endl << endl;
	current_node = which_node;
	waypoint = nodes.col(which_node);
}

void ic_controller::setInitialNextPose(){
	start_commu = false;
	int which_node = (int)policy(0, 3)-1;
	cout << "Brain " << myName << " sets initial next position: node " << policy(0, 3) << " [" << nodes(0, which_node) << ", " << nodes(1, which_node) << "]." << endl << endl; 
	current_node = which_node;
	waypoint = nodes.col(which_node);
}

void ic_controller::setNextPose(){
	start_commu = false;
	int which_node;
	if (current_node == myedge(0)){
		which_node = myedge(1);
	} else if (current_node == myedge(1)) {
		which_node = myedge(0);
	} else {
		cout << "Error: Brain " << myName << " is out of its edge at node " << current_node+1 << "!" << endl << endl;
		exit(EXIT_FAILURE);
	}
	cout << "Brain " << myName << " sets next position: node " << which_node+1 << " [" << nodes(0, which_node) << ", " << nodes(1, which_node) << "]." << endl << endl;
	current_node = which_node;
	waypoint = nodes.col(which_node);
}

// Start communication
void ic_controller::startCommu(){
	start_commu = true;
	setrecord();
	findcurrentNgbr();
	// Query peers' numbers until get fullfilled
	query_state = true;
	cout << "Brain " << myName << " asks for my neighbour Brain ";
	for (int i = 0; i < myngbr[index_node].size(); i++){
		cout << robotName(myngbr[index_node][i]) << " ";
	}
	cout << "'s numbers!" << endl << endl;
	while(query_state){
		if(checkMaster()){
			break;
		}
		publishmsg(1, num_carry, myngbr[index_node]);
		ros::spinOnce();
		checkselfFullfilled();
		pub_rate.sleep();
	}
	cout << "Brain " << myName << " finishes collecting numbers!" << endl << endl;
	// Wait for peers receive my message
	peer_query = true;
	while(peer_query){
		if(checkMaster()){
			break;
		}
		publishmsg(0, num_carry, myngbr[index_node]);
		ros::spinOnce();
		checkpeerFullfilled();
		pub_rate.sleep();
	}
	cout << "All my neighbours finishes collecting numbers!" << endl << endl;
	// Calculate mean
	if (!checkMaster()){
		calcMean();
		start_commu = false;
	}	
}

void ic_controller::waitForSeconds(double i){
	ros::Duration(i).sleep();
}

bool ic_controller::checkMaster(){
	ros::spinOnce();
	if (!state_flag){
		return false;
	} else {
		cout << "Brain " << myName << " receives master's message!" << endl << endl;
		return true;
	}
}

/*******	 Private	*******/
// VRPN Callback
void ic_controller::vrpnCallback(const geometry_msgs::TransformStamped::ConstPtr& msg, int i){
	pose[i](0) = msg->transform.translation.x;
    	pose[i](1) = msg->transform.translation.y;
}

// Load Info
void ic_controller::loadRobotInfo(){
	string filename = "/home/dukerama/hydro_catkin/src/inter_commu/include/inter_commu/robotInfo.txt";
	readMatrix(filename, num_robot, 1, robotName);
	if (show_sys_info){
		cout << "Robot " << robotName.transpose() << " is involved in the experiment!" << endl << endl;
	}
}

void ic_controller::loadNodesInfo(){
	string filename = "/home/dukerama/hydro_catkin/src/inter_commu/include/inter_commu/nodes.txt";
	nodes.resize(2, num_nodes);
	readMatrix(filename, 2, num_nodes, nodes);
	if (show_sys_info){
		cout << "Nodes info is: " << endl << nodes << endl << endl;
	}
}

void ic_controller::loadRobotPoses(){
	string filename = "/home/dukerama/hydro_catkin/src/inter_commu/include/inter_commu/initRobot.txt";
	MatrixXf mat;
	mat.resize(2, num_robot);
	readMatrix(filename, 2, num_robot, mat);
	for (int i = 0; i < num_robot; i++){
		pose.push_back(mat.col(i));
	}
	if (show_sys_info){
		for (int i = 0; i < num_robot; i++){
			cout << "Robot initial pose: [" << pose[i].transpose() << "]." << endl; 
		}
		cout << endl;
	}	
}

void ic_controller::loadInitialNum(){
	string filename = "/home/dukerama/hydro_catkin/src/inter_commu/include/inter_commu/initNum.txt";
	MatrixXf mat;
	mat.resize(num_robot, 1);
	readMatrix(filename, num_robot, 1, mat);
	num_carry = mat(myIndex, 0);
	if (show_sys_info){
		cout << "Brain " << myName << " initial number is: " << num_carry << "." << endl << endl;
		cout << "Goal mean should be: " << mat.mean() << "." << endl << endl;
	}
	recordNumber();	
}

void ic_controller::loadPolicy(){
	string filename = "/home/dukerama/hydro_catkin/src/inter_commu/include/inter_commu/policy.txt";
	MatrixXf mat;
	mat.resize(num_robot, 6);
	readMatrix(filename, num_robot, 6, mat);
	policy.resize(1, 6);
	for (int i = 0; i < num_robot; i++){
		if (myedge(0) == ((int)mat(i, 0)-1) && myedge(1) == ((int)mat(i, 1)-1)){
			policy = mat.row(i);
			break;
		}
	}
	if (show_sys_info){
		cout << "Brain " << myName << " policy is: [" << policy(0,2) << ", " << policy(0,3) << "], [" << policy(0, 4) << ", " << policy(0, 5) << "]^w" << endl << endl; 
	}
}

void ic_controller::readMatrix(string filename, int max_line, int max_col, MatrixXf &mat){
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

int ic_controller::getNumberofLine(string filename){
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

// Initialize functions
void ic_controller::findmyIndex(){
	for (int i = 0; i < num_robot; i++){
		if ((int)robotName(i, 0) == myName){
			myIndex = i;
		} 
	}
	if (show_sys_info){
		cout << "Brain " << myName << " index is: " << myIndex << "." << endl << endl;
	}
}

void ic_controller::findEdges(){
	if (sim){
		loadRobotPoses();
	} else {
		ros::spinOnce();
		if (show_sys_info){
			for (int i = 0; i < num_robot; i++){
				cout << "Robot initial pose: [" << pose[i].transpose() << "]." << endl; 
			}
			cout << endl;
		}
	}

	edges.resize(2, num_robot);
	
	int min_node, smin_node;
	float min_d, smin_d;
	Vector2f mypose, temp;
	for (int ind = 0; ind < num_robot; ind++){
		min_node = 0; smin_node = 0;
		min_d = 100; smin_d = 100;
		mypose = pose[ind];
		for (int i = 0; i < num_nodes; i++){
			temp = nodes.col(i) - mypose;
			if (temp.norm() < smin_d){
				if (temp.norm() < min_d){
					smin_d = min_d;
					smin_node = min_node;
					min_d = temp.norm();
					min_node = i;				
				} else {
					smin_d = temp.norm();
					smin_node = i;
				}		
			}
		}
		if(min_node < smin_node){
			edges(0, ind) = min_node;
			edges(1, ind) = smin_node;
		} else {
			edges(1, ind) = min_node;
			edges(0, ind) = smin_node;
		}
		if(show_sys_info){
			cout << "Brain " << robotName(ind, 0) << " is in edge " << edges(0, ind)+1 << " " << edges(1, ind)+1 << endl << endl; 
		}
		if (ind == myIndex){
			myedge = edges.col(ind);
		}
	}

}

void ic_controller::findmyNeighbour(){
	myngbr.reserve(2);
	vector<int> temp;
	temp.reserve(num_robot);
	count_ngbr = 0;
//	cout << "myedge: " << myedge.transpose() << endl;
//	cout << "edges: " << endl << edges << endl << endl;
	for (int i = 0; i < 2; i++){
		temp.clear();
		for (int j = 0; j < num_robot; j++){
			if(j != myIndex){
//				cout << myedge(i) << " " << edges(0, j) << " " << (myedge(i) == edges(0, j)) << endl;
//				cout << myedge(i) << " " << edges(1, j) << " " << (myedge(i) == edges(1, j)) << endl;
				if ((myedge(i) == edges(0, j)) || (myedge(i) == edges(1, j))){
					temp.push_back(j);
					count_ngbr++;
				}
			}
		}
		myngbr.push_back(temp);
	}
	cout << myngbr[0].size() << " " << myngbr[1].size() << endl << endl;
	if(show_sys_info){
		cout << "I'm Brain " << myName << ", I have " << count_ngbr << " neighbours:" << endl;
		for (int i = 0; i < 2; i++){
			cout << "At node " << myedge[i]+1 << " I have neighbours Brain ";
			for (int j = 0; j < myngbr[i].size(); j++){
				cout << robotName(myngbr[i][j], 0) << " ";
				if (j == (myngbr[i].size() - 1)){cout << endl;}
			}
		}
		cout << endl;
	}
	MatrixXf mynodes;
	mynodes = nodes;
	mynodes.col(myedge[0]) = nodes.col(myedge(0)) + 0.2 * (nodes.col(myedge(1)) - nodes.col(myedge(0)));
	mynodes.col(myedge[1]) = nodes.col(myedge(1)) + 0.2 * (nodes.col(myedge(0)) - nodes.col(myedge(1)));
	nodes = mynodes;
}

void ic_controller::setPub(ros::NodeHandle &nh){
	stringstream ss;
	ss << "Brain" << myName << "/Msg";
	string pub_name =  ss.str();
 	my_pub = nh.advertise<std_msgs::Float32MultiArray>(pub_name.c_str(), 1);
}

void ic_controller::publishmsg(float query, float num, vector<int> ngbr){
	my_msg.data.clear();
	my_msg.data.push_back(query);
	my_msg.data.push_back(num);
	for (int i = 0; i < ngbr.size(); i++){
		my_msg.data.push_back((float)ngbr[i]);
	}
	my_pub.publish(my_msg);
}

void ic_controller::setSub(ros::NodeHandle &nh){
	peer_sub.reserve(count_ngbr);
	for (int i = 0; i < 2; i++){
		for (int j = 0; j < myngbr[i].size(); j++){
			stringstream ss;
			ss << "Brain" << robotName((int)myngbr[i][j], 0) << "/Msg";
			string sub_name = ss.str();
			peer_sub.push_back(nh.subscribe<std_msgs::Float32MultiArray>(sub_name.c_str(), 1, boost::bind(&ic_controller::peerCallback, this, _1, myngbr[i][j])));
		}
	}

	peers_info.reserve(num_robot);	
}

void ic_controller::peerCallback(const std_msgs::Float32MultiArray::ConstPtr& array, int i){
	if (start_commu){
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
		// If in query state;
		if (query_state){
			if ( checkmsg4me() ){
				data_record[i] = peers_info[i][1];
				selfrecord[i] = true;
			}
		} else if (peer_query) {
			if ((int)(peers_info[i][0]+0.2) == 0){
				peerrecord[i] = true;			
			}
		}
	}
}

bool ic_controller::checkmsg4me(){
	bool temp = false;
	for (int i = 0; i < to_who.size(); i++){
		if (myIndex == to_who[i]){
			temp = true;
		}
	}
	return temp;
}

void ic_controller::setrecord(){
	for (int i = 0; i < num_robot; i++){
		selfrecord[i] = false;
		peerrecord[i] = false;
	}
}

void ic_controller::checkselfFullfilled(){
	bool temp = true;
	for (int i = 0; i < myngbr[index_node].size(); i++){
		temp = temp && selfrecord[myngbr[index_node][i]];
	}
	query_state = !temp;
}

void ic_controller::checkpeerFullfilled(){
	bool temp = true;
	for (int i = 0; i < myngbr[index_node].size(); i++){
		temp = temp && peerrecord[myngbr[index_node][i]];
	}
	peer_query = !temp;
}

void ic_controller::findcurrentNgbr(){
	if (current_node == myedge.minCoeff()){
		index_node = 0;
	} else if (current_node == myedge.maxCoeff()){
		index_node = 1;
	} else {
		cout << "Error: current node " << current_node + 1 << " is not in my edge list! " << endl << endl;
		exit(EXIT_FAILURE);
	}
}

void ic_controller::calcMean(){
	float temp;
	temp = num_carry;
	cout << "Brain " << myName << " former carries number: " << num_carry << "." << endl << endl;
	for (int i = 0; i < myngbr[index_node].size(); i++){
		temp = temp + data_record[myngbr[index_node][i]];
	}
	num_carry = temp / (float)(myngbr[index_node].size() + 1);
	cout << "Brain " << myName << " current carries number: " << num_carry << "." << endl << endl;
	recordNumber();
//	waitForEnter();
}

void ic_controller::recordNumber(){
	outfile_number << num_carry << "\n";
}

void ic_controller::closeRecord(){
	outfile_number.close();
}

void ic_controller::masterCallback(const std_msgs::Float32MultiArray::ConstPtr& array){
	for (std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it){
		if ((int)(*it + 0.1) == 1){ state_flag = true; }
	}
}

#endif
