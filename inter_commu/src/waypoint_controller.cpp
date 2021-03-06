#ifndef __WAYPOINT_CONTROLLER_cpp___
#define __WAYPOINT_CONTROLLER_cpp___

#include "waypoint_controller.h"

#define PI 3.14159265

using namespace std;
using namespace Eigen;

/********	Public	      ********/
waypoint_controller::waypoint_controller(ros::NodeHandle &nh):rotate_pid(1.0, 0.2, 0.1, 0.5, -0.5 , Base_Orientation_Error_Tolerance/180*PI), 
							      forward_pid(1.0, 0.2, 0, 0.2, -0.2, Waypoint_Error_Tolerance){
	cout << "Initialize waypoint controller..." << endl;
	vrpn_sub = nh.subscribe<geometry_msgs::TransformStamped>("Brain5Camera/pose", 1, &waypoint_controller::vrpnCallback, this);

	// Load Base info
	show_Base_info = true;
	loadBase();

  	std::string publishname;
  	publishname = "Brain5/cmd_vel";
  	base_pub = nh.advertise<geometry_msgs::Twist>(publishname.c_str(), 1);
}

void waypoint_controller::goWaypoint(Vector2f w_hat_order){
	w_hat = w_hat_order;

	// Rotate base
	PID_rotate();

	// Move base
	PID_forward(false);
}

void waypoint_controller::test_goWaypoint(){
	Vector2f goal;
	goal << 0, 0;
	goWaypoint(goal);
}

void waypoint_controller::waitForEnter(){
	cout << "Please press any key to continue... ";
	char enter;
	cin >> enter;
	cin.clear();
	cin.sync();
}

/********	Private	      ********/
void waypoint_controller::vrpnCallback(const geometry_msgs::TransformStamped msg){
    q_h[0] =msg.transform.rotation.x;
    q_h[1] =msg.transform.rotation.y;
    q_h[2] =msg.transform.rotation.z;
    q_h[3] =msg.transform.rotation.w;
  
    Eigen::Quaternionf quat_h(q_h[3], q_h[0], q_h[1], q_h[2]);

    Rot_h2w = quat_h.matrix();

    ohand_world[0] = msg.transform.translation.x;
    ohand_world[1] = msg.transform.translation.y;
    ohand_world[2] = msg.transform.translation.z;
}

void waypoint_controller::loadBase(){
	string filename = "/home/dukerama/hydro_catkin/src/inter_commu/include/BaseCalib/R_sb.txt";
	R_sb = loadMatrix(2, 2, filename);
	if (show_Base_info){
		cout << "R_sb: " << endl << R_sb << endl << endl;
	}
}

MatrixXf waypoint_controller::loadMatrix(int maxline, int maxcol, string filename){
	MatrixXf mat;
	mat.resize(maxline, maxcol);
	mat.setZero();
	ifstream file;
  	file.open(filename.c_str());
  	if (!file.is_open()){
    		cout << "No file!\n";
    		return mat; 
  	}
	for (int line = 0; line < maxline; line++){
		for (int col = 0; col < maxcol; col++){
			file >> mat(line, col);	
		}
	}
	file.close();
	return mat;
}

void waypoint_controller::getBasePose(){
	ros::spinOnce();
	R_hw = matrix2d(1, 2, Rot_h2w);
	p_hand(0) = ohand_world(0);	p_hand(1) = ohand_world(1);
	// Solve current base pose
	R_bw = R_hw * R_sb.transpose();
}

Matrix2f waypoint_controller::matrix2d(int index1, int index2, Matrix3f mat){
	Matrix2f mat_new;
	mat_new(0, 0) = mat(index1-1, index1-1);
	mat_new(0, 1) = mat(index1-1, index2-1);
	mat_new(1, 0) = mat(index2-1, index1-1);
	mat_new(1, 1) = mat(index2-1, index2-1);
	return mat_new;
}
void waypoint_controller::calcRotateError(){
	getBasePose();
	ax_goal = w_hat - p_hand;
	ax_base = R_bw.transpose() * ax_goal;
	err_rad = atan2(ax_base(1), ax_base(0));
	if (err_rad > PI/2){
		err_rad = err_rad - PI;
	} else if (err_rad < -PI/2){
		err_rad = err_rad + PI;
	}
}

void waypoint_controller::PID_rotate(){
	bool flag = false;
	calcRotateError();
	if (fabs(err_rad/PI*180) < Base_Orientation_Error_Tolerance){
		flag = true;
	}
	while (!flag){
		detectFreeze();
		rotate_pid.setError(err_rad);
        	u_msg.angular.z = rotate_pid.computePID();
     		u_msg.linear.x = 0;
      		base_pub.publish(u_msg);
      		cout << "Base Error: " << err_rad/PI*180 << " In radian: " << err_rad << endl;
      		ros::Duration(0.1).sleep();
		calcRotateError();
		if (fabs(err_rad/PI*180) < Base_Orientation_Error_Tolerance){
			stopBase();
			cout << "Rotate goal achieved with error " << err_rad/PI*180 << " degree." << endl << endl;
			flag = true;
		}
	}
	rotate_pid.clearhistory();
}

void waypoint_controller::calcForwardError(){
	getBasePose();
	bearing = R_bw.transpose() * (w_hat - p_hand);
	bearing_angle = atan(bearing(1)/bearing(0))*180/PI;
}

void waypoint_controller::PID_forward(bool rotate){
	bool flag = false;
	calcForwardError();
	if (fabs(bearing.norm()) < Waypoint_Error_Tolerance){
		flag = true;
	}
	double start_time = ros::Time::now().toSec();
	double duration = 0;
	while (!flag){
		detectFreeze();
		forward_pid.setError(bearing(0));
        	u_msg.linear.x = forward_pid.computePID();
		if (rotate){
			rotate_pid.setError(atan2(bearing(1), bearing(0)));
     			u_msg.angular.z = rotate_pid.computePID();
		} else {
			u_msg.angular.z = 0;
		}
      		base_pub.publish(u_msg);

		calcForwardError();
		// Test biased bearing angle
		cout << "Biased bearing angle: " << bearing_angle << endl;
      		cout << "Forward Error: " << bearing.norm() << " (m) " << endl;
      		ros::Duration(0.1).sleep();

		duration = ros::Time::now().toSec() - start_time;
//		cout << "Duration: " << duration << endl;
          	if (duration > 10.0) {
            		cout << "Forward time out with error " << bearing.norm() << " meters." << endl << endl;
            		flag = true;
          	}
		if (fabs(bearing_angle) > 80){
			stopBase();
			cout << "Forward goal achieved with error " << bearing.norm() << " meters." << endl << endl;
			flag = true;
		}
		if (fabs(bearing.norm()) < Waypoint_Error_Tolerance){
			stopBase();
			cout << "Forward goal achieved with error " << bearing.norm() << " meters." << endl << endl;
			flag = true;
		}
	}
	forward_pid.clearhistory();
	if (rotate) {
		rotate_pid.clearhistory();
	}
}

void waypoint_controller::stopBase()
{
      u_msg.angular.z = 0;
      u_msg.linear.x = 0;
      base_pub.publish(u_msg);
}

void waypoint_controller::backwardBase()
{
	// Check if the waypoint is in front or in backwards
	getBasePose();
	Vector2f vec = R_bw.transpose() * (w_hat - p_hand);
	bool flag;
	if (vec(0) > 0){	// waypoint is in the front of robot
		flag = true;
	} else {
		flag = false;
	}

      	u_msg.angular.z = 0;
	if (flag) {
      		u_msg.linear.x = -0.1;
	} else {
		u_msg.linear.x = 0.1;
	}
      	base_pub.publish(u_msg);
      	ros::Duration(0.5).sleep();
      	stopBase();
}

bool waypoint_controller::detectFreeze(){
	getBasePose();
	Vector2f p_hand_old = p_hand;
	ros::Duration(0.1).sleep();
	getBasePose();
	bool freeze = false;
	if ((p_hand_old - p_hand).norm() < Freeze_Tolerance){
		cout << "Freeze detected!" << endl;
		stopBase();
		freeze = true;
	}
	while ((p_hand_old - p_hand).norm() < Freeze_Tolerance){
		getBasePose();	
	}
	if(freeze){
		cout << "Freeze released!" << endl;
		freeze = false;
	}
}

#endif
