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

	// Initialize servo
	sendServoSpeed(5);
	initialServoTarget();

	// Load Base info
	show_Base_info = true;
	loadBase();
	
	ex << 1, 0;

  	std::string publishname;
  	publishname = "Brain5/cmd_vel";
  	base_pub = nh.advertise<geometry_msgs::Twist>(publishname.c_str(), 1);
}

void waypoint_controller::goWaypoint(Vector2f w_hat_goal, Matrix2f Rg_hat_goal){
	w_hat = w_hat_goal;
	Rg_hat = Rg_hat_goal;

//	// Test
//	getBasePose();
//	waitForEnter();
//	cout << "w_hat: " << w_hat.transpose() << endl;
//	cout << "p_base: " << p_base.transpose() << endl;
//	Vector2f shift = w_hat - p_base;
//	cout << "shift: " << shift.transpose() << endl << endl;

	// Transform order to base frame;
	trans2base();
	doable = calcAngle();
	while(!doable){
		// Forward or Backward a little
		backwardBase();
		trans2base();
		doable = calcAngle();
	}

	cout << "Angle base and servo should rotate: servo - " << angle_s << " (deg) " << "base - " << theta_b << " (deg)" << endl; 
	// Move servo
	float ms = (-angle_s/180*500 + 1500) * 4;
	sendServoTarget(ms);
	// Rotate base
	PID_rotate();
	// Move base
	PID_forward(false);

	// Report tracking error;
	getBasePose();
	cout << "desired waypoint: " << w_hat.transpose() << endl << endl;
	cout << "realized waypoint: " << p_hand.transpose() << endl << endl;
	cout << "desired hand orient: " << endl << Rg_hat << endl << endl;
	cout << "realized hand orient: " << endl << Rot_h2w << endl << endl;
}

void waypoint_controller::testServo(){
	// Test servo
	float servo_target;
	cout << "Input target angle for servo: ";
	cin >> servo_target;

	float ms = (-servo_target/180*500 + 1500) * 4;
	sendServoTarget(ms);
}

void waypoint_controller::testVrpn(){
	cout << "Type any key to get response from VRPN: ";
	float enter;
	cin >> enter;
	cin.clear();
	cin.sync();
	ros::spinOnce();
	cout << "Hand center in the world: " << ohand_world.transpose() << endl;
	cout << "Hand orient in the world: " << endl << Rot_h2w << endl;
}

void waypoint_controller::testBaseInfo(){
	cout << "Type any key to test Base info: ";
	float enter;
	cin >> enter;
	cin.clear();
	cin.sync();
	// Get realtime base pose
	getBasePose();	// R_bw and p_base is updated;
	
	// Test the quality of base pose estimation
	cout << "Base position: " << p_base.transpose() << endl;
	cout << "Base orientation: " << endl << R_bw << endl << endl;	
}

void waypoint_controller::testCalcAngle(){
	w << 0.05, 0;
	float theta_g = 30;
	Rg = angle2rot(theta_g);
	calcAngle();

	cout << "angle_a: " << angle_a << endl;
	cout << "angle_rot: " << angle_rot << endl;
	cout << "angle_b1: " << angle_b1 << endl;
	cout << "angle_b2: " << angle_b2 << endl;
	cout <<	"angle_s: " << angle_s << endl;
}

void waypoint_controller::testRotatePID(){
	ax_goal << 1, 0;
	PID_rotate();
}

void waypoint_controller::testForwardPID(){
	ax_goal << 1, 0;
	PID_rotate();

	getBasePose();
	Vector2f shift;
	shift << 0.08, 0;
	w_hat = p_hand + shift;
	PID_forward(true); // Without rotation
}

void waypoint_controller::testgoWaypoint(){
	getBasePose();
	Vector2f shift;
	shift << -0.03, -0.00;
	Vector2f w_hat_goal;
	w_hat_goal = p_base + shift;
	Matrix2f Rg_hat_goal;
	Rg_hat_goal << 0.8660, 0.5000,
		      -0.5000, 0.8660;

	w_hat = w_hat_goal;
	Rg_hat = Rg_hat_goal;
	// Transform order to base frame;
	trans2base();
	doable = calcAngle();
	while(!doable){
		// Forward or Backward a little
		backwardBase();	
		trans2base();
		doable = calcAngle();
	}

	cout << "Angle base and servo should rotate: servo - " << angle_s << " (deg) " << "base - " << theta_b << " (deg)" << endl; 
	// Move servo
	float ms = (-angle_s/180*500 + 1500) * 4;
	sendServoTarget(ms);
	// Rotate base
	PID_rotate();

//	cout << "current base orientation: " << endl << R_bw << endl << endl;
//	cout << "current hand to waypoint direction: " << (w_hat - p_hand).transpose() << endl << endl;

//	// Move base
	PID_forward(false);

	// Report tracking error;
	getBasePose();
	cout << "desired waypoint: " << w_hat.transpose() << endl << endl;
	cout << "realized waypoint: " << p_hand.transpose() << endl << endl;
	cout << "desired hand orient: " << endl << Rg_hat << endl << endl;
	cout << "realized hand orient: " << endl << Rot_h2w << endl << endl;
}

void waypoint_controller::waitForEnter(){
	cout << "Please press any key to continue... ";
	char enter;
	cin >> enter;
	cin.clear();
	cin.sync();
}

/********	Private	      ********/
void waypoint_controller::sendServoSpeed(float speed){
    int fd = maestroConnect();
    maestroSetSpeed(fd, 0, speed);
    close(fd); 
}

void waypoint_controller::initialServoTarget(){
    int fd = maestroConnect();
    maestroSetTarget(fd, 0, 6000);
    close(fd); 
}

void waypoint_controller::getServoPosition(){
    int fd = maestroConnect();
    servo_position = maestroGetPosition(fd, 0)/4;
    close(fd);
    servo_angle = -(servo_position - 1500)/500*180;
    if (servo_position == 0){
	cout << "Error! Servo position is 0 ms!" << endl << endl; 
    }
}

void waypoint_controller::sendServoTarget(float ms){
    int fd = maestroConnect();
    maestroSetTarget(fd, 0, ms);
    while (GetMovingState(fd,0)) {}
    close(fd);
}

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

    Rot_c2w = Rot_h2w * Rot_c2h;
    Rot_w2c = Rot_c2w.transpose();
    ocam_world = Rot_h2w * ocam_hand + ohand_world;
}

void waypoint_controller::loadBase(){
	string filename = "/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/BaseCalib/Hc2m_left.txt";
	Hc2m_left = loadMatrix(4, 4, filename);
	if (show_Base_info){
		cout << "Hc2m_left: " << endl << Hc2m_left << endl << endl;
	}

	filename = "/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/BaseCalib/Hc2m_right.txt";
	Hc2m_right = loadMatrix(4, 4, filename);
	if (show_Base_info){
		cout << "Hc2m_right: " << endl << Hc2m_right << endl << endl;
	}

	filename = "/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/BaseCalib/r_sb.txt";
	r_sb = loadMatrix(2, 1, filename);
	if (show_Base_info){
		cout << "r_sb: " << r_sb.transpose() << endl << endl;
	}

	filename = "/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/BaseCalib/R_sb.txt";
	R_sb = loadMatrix(2, 2, filename);
	if (show_Base_info){
		cout << "R_sb: " << endl << R_sb << endl << endl;
	}

	filename = "/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/BaseCalib/r_hs.txt";
	r_hs = loadMatrix(2, 1, filename);
	if (show_Base_info){
		cout << "r_hs: " << r_hs.transpose() << endl << endl;
	}

  	for (int line = 0; line < 3; line++){
      		ocam_hand(line) = (Hc2m_left(line, 3) + Hc2m_right(line, 3))/2/1000;
  	}

  	for (int line = 0; line < 3; line++){
      		for (int col = 0; col < 3; col++){
          		Rot_c2h(line, col) = Hc2m_left(line, col);
      		}
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
	getServoPosition();
	// Transform data into 2d format
	R_stheta = angle2rot(servo_angle);
	R_hw = matrix2d(1, 2, Rot_h2w);
	p_hand(0) = ohand_world(0);	p_hand(1) = ohand_world(1);
	// Solve current base pose
	R_bw = R_hw * R_stheta.transpose() * R_sb.transpose();
	p_base = p_hand - R_bw * r_sb - R_bw * R_sb * R_stheta * r_hs;
//	// Test
//	cout << "ohand_world: " << ohand_world.transpose() << endl;
//	cout << "Rot_h2w: " << endl << Rot_h2w << endl;
//	cout << "p_hand: " << p_hand.transpose() << endl;
//	cout << "R_bw: " << endl << R_bw << endl;
//	cout << "R_stheta: " << endl << R_stheta << endl;
}

Matrix2f waypoint_controller::angle2rot(float angle){
	Matrix2f mat;
	float vcos = cos(angle * PI / 180.0);
	float vsin = sin(angle * PI / 180.0);
	mat << vcos, vsin,
	      -vsin, vcos;
	return mat;
}

Matrix2f waypoint_controller::matrix2d(int index1, int index2, Matrix3f mat){
	Matrix2f mat_new;
	mat_new(0, 0) = mat(index1-1, index1-1);
	mat_new(0, 1) = mat(index1-1, index2-1);
	mat_new(1, 0) = mat(index2-1, index1-1);
	mat_new(1, 1) = mat(index2-1, index2-1);
	return mat_new;
}

void waypoint_controller::trans2base(){
	// Get realtime base pose
	getBasePose();	// R_bw and p_base is updated;
	
	// Test the quality of base pose estimation
//	cout << "Base position: " << p_base.transpose() << endl;
//	cout << "Base orientation: " << endl << R_bw << endl << endl;

	w = R_bw.transpose() * (w_hat - p_base);
	Rg = R_bw.transpose() * Rg_hat;	
}

bool waypoint_controller::calcAngle(){
	Vector2f temp;
	temp = w - Rg * r_hs;
	a(0) = temp(1); a(1) = temp(0);
	b = r_sb(1);

	if (a.norm() > b){
		angle_a = atan(a(1) / a(0)) * 180 / PI;
		if (a(0) < 0) {
			angle_a = angle_a + 180;
		}
		angle_rot = acos(b / a.norm()) * 180 / PI;
		angle_b1 = angle_a + angle_rot;
		angle_b2 = angle_a - angle_rot;
		theta_b = angle_b1;
		if (fabs(angle_b1) > fabs(angle_b2)){
			theta_b = angle_b2;
		}
		R_b_delta = angle2rot(theta_b);
		R_s_delta = R_sb.transpose() * R_b_delta.transpose() * Rg;
		angle_s = acos(R_s_delta(0, 0)) * 180 / PI;
		if (R_s_delta(0, 1) < 0) {
			angle_s = -angle_s;
		}
		R_basegoal = R_bw * R_b_delta;
		ax_goal = R_basegoal * ex;
		return true;
	} else {
		cout << "Unachievable position!" << endl;
		return false;
	}
}

void waypoint_controller::calcRotateError(){
	getBasePose();
	ax_base = R_bw.transpose() * ax_goal;
	err_rad = atan2(ax_base(1), ax_base(0));
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
	// Test biased bearing angle
	cout << "Biased bearing angle: " << bearing_angle << endl;
}

void waypoint_controller::PID_forward(bool rotate){
	bool flag = false;
	calcForwardError();
	if (fabs(bearing.norm()) < Waypoint_Error_Tolerance){
		flag = true;
	}
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
      		cout << "Forward Error: " << bearing.norm() << " (m) " << endl;
      		ros::Duration(0.2).sleep();
		calcForwardError();
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
