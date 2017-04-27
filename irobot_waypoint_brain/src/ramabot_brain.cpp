#ifndef __RAMABOT_BRAIN_cpp____
#define __RAMABOT_BRAIN_cpp____

#include "ramabot_brain.h"

using namespace std;

//*********************** RAMA_COMMANDER CLASS ***********************//
rama_commander::rama_commander(){
  wpt_count = 0;
  goal_target = 0;
  fx = 8;
  fy = 8;
}

void rama_commander::loadWaypoints()
{
  int line, col;
  ifstream file;
  file.open("/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/Config/Waypoints.txt");
    
  if (!file.is_open()){
    cout << "No Waypoints.txt file in the /include/Config folder!\n";
    return; 
  }

  for(line = 0; line < NUM_WPT; line++){
    for (col = 0; col < 3; col++){
      file >> Horizon(line, col);
    }
  }
  
  file.close();
}

void rama_commander::loadTargets()
{
  int line, col;
  ifstream file;
  file.open("/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/Config/InitTargets.txt");
    
  if (!file.is_open()){
    cout << "No InitTargets.txt file in the /include/Config folder!\n";
    return; 
  }

  for(line = 0; line < NUM_TARGET; line++){
    for (col = 0; col < 3; col++){
      file >> Targets(line, col);
    }
  }
  
  file.close();
}

Eigen::Vector3f rama_commander::sendWaypoints(){
  if (wpt_request == 1 && wpt_done == 0 && wpt_count < NUM_WPT){
    printf("Commander : Request received! Please go to the %dth waypoint [%f, %f, %f]...\n", wpt_count+1, Horizon(wpt_count, 0), Horizon(wpt_count, 1), Horizon(wpt_count, 2));
    
//    waypoint(0) = Horizon(wpt_count, 0); waypoint(1) = Horizon(wpt_count, 1); waypoint(2) = Horizon(wpt_count, 2);
    goal_msg.x = Horizon(wpt_count, 0); goal_msg.y = Horizon(wpt_count, 1); goal_msg.z = Horizon(wpt_count, 2);   
    target_msg.x = Targets(goal_target, 0); target_msg.y = Targets(goal_target, 1); target_msg.z = Targets(goal_target, 2);
    
    cam_goal = Horizon.row(wpt_count);
    tgt = Targets.row(goal_target);

    cout << "cam_goal: " << cam_goal.transpose() << endl;
    cout << "tgt: " << tgt.transpose() << endl;
  }
}

int rama_commander::check_all_sent(){
  if (wpt_count >= NUM_WPT) {return 1;}
  else {return 0;}
}

void rama_commander::nextWaypoints(){
  wpt_count++;
}

cv::Mat rama_commander::downsample(cv::Mat image)
{
  cv::Size Size = image.size();
  int rows = Size.height;
  int cols = Size.width;
  cv::Mat new_img;
  cv::Size newSize(cols/fx, rows/fy);
  resize(image, new_img, newSize);
  return new_img;
}

cv::Mat rama_commander::findCircle(cv::Mat image)
{
  cv::Mat imgHSV;
  cv::cvtColor(image, imgHSV, cv::COLOR_BGR2HSV);
  cv::Mat imgThresholded;

//  Orange_MIN = [6, 50, 50], Orange_MAX = [14, 255, 255]
  inRange(imgHSV, cv::Scalar(6, 50, 50), cv::Scalar(14, 255, 255), imgThresholded);
//  Some Morphological opening stuff, removing small objects
  erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2,2)));
  dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2,2)));
//  Some Morphological opening stuff, filling small holes
  dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2,2)));
  erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2,2)));

  cv::Size size = image.size();
  cv::Mat mask = cv::Mat::zeros(size.height, size.width, CV_8U);
  mask(cv::Rect(0, size.height/2, size.width, size.height-size.height/2)) = 1;
  cv::Mat outputimg;
  imgThresholded.copyTo(outputimg, mask);
//  std::cout << size.height/3 << " " << size.height - size.height/3 << std::endl;

  cv::Mat nonZeros;
  cv::findNonZero(outputimg, nonZeros);
  cv::Size sssize = nonZeros.size();
  std::cout << sssize.height << " " << sssize.width << std::endl; 

//  return imgThresholded;
  return outputimg;
}

//************************* RAMA_COMMANDER for CLUSTER DP *************************//

void rama_commander::loadClusterTargets()
{
  int line, col;
  ifstream file;
  file.open("/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/Config/ClusterDP/InitTargets.txt");
    
  if (!file.is_open()){
    cout << "No InitTargets.txt file in the /include/Config/ClusterDP folder!\n";
    return; 
  }

  for(line = 0; line < NUM_TARGET; line++){
    for (col = 0; col < 3; col++){
      file >> Targets(line, col);
    }
  }
  
  file.close();
}

void rama_commander::loadEntryList()
{
  int line, col, number_of_lines;

  int index_target;
  for (index_target = 1; index_target < NUM_TARGET+1; index_target++)
  {
    ifstream file;
    char name[255];
    std::stringstream ss;
    std::string filename;
    sprintf(name, "/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/Config/ClusterDP/EntryList%d.txt", index_target);
    ss << name;
    ss >> filename;
//    std::cout << filename << std::endl;
    number_of_lines = getNumberofLine(filename);
//    std::cout << number_of_lines << std::endl;

    file.open(filename.c_str());   
    if (!file.is_open()){
      printf("No such a file %s !\n", filename.c_str());
      return; 
    }

    Eigen::MatrixXf temp(number_of_lines, 3);
    for(line = 0; line < number_of_lines; line++){
      for (col = 0; col < 3; col++){   
        file >> temp(line, col);
      }
    } 
    file.close();
    EntryList[index_target] = temp;
  }
}

void rama_commander::loadClusterWaypointList()
{
  int line, col, number_of_entries, number_of_waypoints;

  int index_target;
  for (index_target = 1; index_target < NUM_TARGET+1; index_target++)
  {
    ifstream file;
    char name[255];
    std::stringstream ss;
    std::string filename;
    sprintf(name, "/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/Config/ClusterDP/EntryList%d.txt", index_target);
    ss << name;
    ss >> filename;
//    std::cout << filename << std::endl;
    number_of_entries = getNumberofLine(filename);
//    std::cout << number_of_lines << std::endl;
    
    int index_entry;
    for (index_entry = 1; index_entry < number_of_entries+1; index_entry++)
    {
      ifstream file;
      char name[255];
      std::stringstream ss;
      std::string filename;
      sprintf(name, "/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/Config/ClusterDP/Waypoint0%d0%d.txt", index_target, index_entry);
      ss << name;
      ss >> filename;
      file.open(filename.c_str());   
      if (!file.is_open()){
        printf("No such a file %s !\n", filename.c_str());
        return; 
      }
//      std::cout << filename << std::endl;
      number_of_waypoints = getNumberofLine(filename);
      Eigen::MatrixXf temp(number_of_waypoints, 4); // Real file has 3 columns, although the third column is useless!!!
      for(line = 0; line < number_of_waypoints; line++){
        for (col = 0; col < 4; col++){   
          file >> temp(line, col);
        }
      } 
      file.close();
      WaypointList[index_target][index_entry] = temp;
    }
  }
}

Eigen::VectorXi rama_commander::chooseNearestEntry()
{
  flag_getCurrentPosition = 0;
  while (flag_getCurrentPosition == 0)
  {  ros::spinOnce(); ros::Duration(0.01).sleep(); }

  int index_target, ii;
  float min_dist = 10.0;
  Eigen::VectorXf ocam2entry(2);
  Eigen::VectorXi output(2);
  Eigen::MatrixXf temp;
  for (index_target = 1; index_target < NUM_TARGET+1; index_target++)
  {
    temp = EntryList[index_target];
    for (ii = 0; ii < temp.rows(); ii++)
    {
      ocam2entry(0) = temp(ii, 0) - ocam_world(0);
      ocam2entry(1) = temp(ii, 1) - ocam_world(1);
      if (ocam2entry.norm() < min_dist)
      {
        min_dist = ocam2entry.norm();
        output(0) = index_target;
        output(1) = ii+1;
      }
    }
  }
  flag_getCurrentPosition = 0;
//  std::cout << output(0) << " " << output(1) << " " << min_dist << std::endl;
  return output; // output(0) = min_index_target, output(1) = min_index_entry;
}

void rama_commander::getClusterWaypoints()
{
  Eigen::VectorXi chosen(2);
  int row;
  if (wpt_request == 1 && wpt_done == 0 && wpt_count == 0)
  {
    chosen = chooseNearestEntry();
    int chosen_target = chosen(0);
    int chosen_entry = chosen(1);
//    std::cout << chosen_target << " " << chosen_entry << std::endl;
    chosen_waypoints = WaypointList[chosen_target][chosen_entry];

    int ii, col;
    for (ii = 0; ii < chosen_waypoints.rows(); ii++)
    {
      for (col = 0; col < 4; col++)
      {
        outfile_entry << chosen_waypoints(ii, col) << " ";
      }
      outfile_entry << "\n";
    }
    outfile_entry.close();

    no_waypoints = chosen_waypoints.rows();
//    std::cout << "WaypointList[Target = " << chosen_target << "][Entry = " << chosen_entry << "] = " << "\n";
//    for (row = 0; row < chosen_waypoints.rows(); row++)
//    { std::cout << chosen_waypoints(row, 0) << " " << chosen_waypoints(row, 1) << " " << chosen_waypoints(row, 2) << " " << chosen_waypoints(row, 3) << "\n"; }
  }

  if (wpt_request == 1 && wpt_done == 0 && wpt_count < chosen_waypoints.rows()){
    goal_target = int(chosen_waypoints(wpt_count, 3)+0.5) - 1;
    printf("Commander : Request received! Please go to waypoint [%f, %f, %f] and detect Target %d...\n", chosen_waypoints(wpt_count, 0), chosen_waypoints(wpt_count, 1), chosen_waypoints(wpt_count, 2), goal_target+1); 
    goal_msg.x = chosen_waypoints(wpt_count, 0); goal_msg.y = chosen_waypoints(wpt_count, 1); goal_msg.z = chosen_waypoints(wpt_count, 2);   
    target_msg.x = Targets(goal_target, 0); target_msg.y = Targets(goal_target, 1); target_msg.z = Targets(goal_target, 2);  
  }

}

int rama_commander::cluster_check_all_sent(){
  if (wpt_count < no_waypoints) {return 0;}
  else {return 1;}
}


//************************* RAMA_ROBOT CLASS ****************************//

rama_robot::rama_robot(){
  robot_wpt_count = 0;
}

void rama_robot::waitForEnter(){
  char Enter;
  cin >> Enter;
}

void rama_robot::rqstWaypoints(){
  wpt_request = 1;
  wpt_done = 0;
  printf("Robot : Request for waypoint!\n");
}

void rama_robot::sendWptDone(){
  wpt_request = 0;
  wpt_done = 1;
  geometry_msgs::Point state_msg;
  state_msg.x = wpt_request; state_msg.y = wpt_done;
  robot_state_pub.publish(state_msg);
}

void rama_robot::startServo(){
  wpt_request = 0;
  wpt_done = 0;
  geometry_msgs::Point state_msg;
  state_msg.x = wpt_request; state_msg.y = wpt_done;
  robot_state_pub.publish(state_msg);
}

void rama_robot::goingWaypoint()
{
  waypoint_pub.publish(goal_msg);
  pid_state = 1;
  std::cout << "Robot : Going waypoint!\n";
  while(pid_state == 1)
  {
    ros::spinOnce();
  }
}

void rama_robot::localrecordImages(int count, cv::Mat image_left, cv::Mat image_right)
{
    //show image to image window
    imshow("right",image_right);
    cv::waitKey(30);
    imshow("left",image_left);
    cv::waitKey(30);
	
    //write image to file
    char filename_left[512];
    sprintf( filename_left, "/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/images/left%d.bmp", count );
    char filename_right[512];
    sprintf( filename_right, "/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/images/right%d.bmp", count );
    imwrite(filename_left,image_left);
    imwrite(filename_right,image_right); 
}


void rama_robot::recordImages(int count, cv::Mat image_left, cv::Mat image_right)
{
    //show image to image window
    imshow("right",image_right);
    cv::waitKey(30);
    imshow("left",image_left);
    cv::waitKey(30);

    int which_target = (int)(chosen_waypoints(robot_wpt_count-1, 3)+0.5);
	
    //write image to file
    char filename_left[512];
    sprintf( filename_left, "/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/images/left0%d0%d.bmp", robot_wpt_count, which_target );
    char filename_right[512];
    sprintf( filename_right, "/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/images/right0%d0%d.bmp", robot_wpt_count, which_target );
    imwrite(filename_left,image_left);
    imwrite(filename_right,image_right); 

}

void rama_robot::detectCircles(int count, cv::Mat image_left, cv::Mat image_right)
{
        cv::Mat detected_edges;
	cv::Mat image_gray;

	cv::cvtColor(image_left, image_gray, CV_BGR2GRAY);

//	/// Reduce noise with a kernel 3x3	
        cv::GaussianBlur(image_gray,detected_edges,cv::Size(5,5),0,0);
	
	std::vector<cv::Vec3f> circles;
        cv::HoughCircles( detected_edges, circles, CV_HOUGH_GRADIENT, 1, detected_edges.cols/10, 120, 20, 0, 30);

	for (size_t i = 0; i < circles.size(); i++ )
		{
			//std::cout <<"detecting " << std::endl;
			cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			circle( image_left, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0 );
			circle( image_left, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0 );
			outfile_left << circles[i][0] << " ";
			outfile_left << circles[i][1] << " ";
			outfile_left << circles[i][2] << " ";
		}
	outfile_left << "\n";

	char filename_detector_left[512];
	sprintf( filename_detector_left, "/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/detector/left%d.bmp", count );
	imwrite(filename_detector_left,image_left);	

//******************* Right Camera *********************//

	cv::cvtColor(image_right, image_gray, CV_BGR2GRAY);

//	/// Reduce noise with a kernel 3x3	
        cv::GaussianBlur(image_gray,detected_edges,cv::Size(5,5),0,0);
	
        cv::HoughCircles( detected_edges, circles, CV_HOUGH_GRADIENT, 1, detected_edges.cols/10, 120, 20, 0, 30);

	for (size_t i = 0; i < circles.size(); i++ )
		{
			//std::cout <<"detecting " << std::endl;
			cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			circle( image_right, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0 );
			circle( image_right, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0 );
			outfile_right << circles[i][0] << " ";
			outfile_right << circles[i][1] << " ";
			outfile_right << circles[i][2] << " ";
		}
	outfile_right << "\n";

	char filename_detector_right[512];
	sprintf( filename_detector_right, "/home/dukerama/hydro_catkin/src/irobot_waypoint_brain/include/detector/right%d.bmp", count );
	imwrite(filename_detector_right,image_right);	
}

void rama_robot::recordCamPose()
{
  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce();

  for (int i=0;i<3;i++)
  {
    outfile_cam << ocam_world(i) << " ";
  }
  for (int i=0;i<4;i++)
  {
    outfile_cam << q_cam(i) << " ";
  }
  outfile_cam << "\n";

  for (int i=0;i<3;i++)
  {
    outfile_hand << ohand_world(i) << " ";
  }
  for (int i=0;i<4;i++)
  {
    outfile_hand << q_h(i) << " ";
  }
  outfile_hand << "\n";
}

void rama_robot::adjustOrientation()
{
  wpt_request = 0;
  wpt_done = 0;
  waypoint_pub.publish(goal_msg);
  robot_target_pub.publish(target_msg);

  geometry_msgs::Point state_msg;
  state_msg.x = wpt_request; state_msg.y = wpt_done;

  robot_state_pub.publish(state_msg);
  robot_state_pub.publish(state_msg);
  robot_state_pub.publish(state_msg);

  ori_pid_state = 0;
  std::cout << "Adjusting orientation of camera and robot base..." << "\n" << std::endl; 
  while (ori_pid_state == 0)
  { ros::spinOnce(); }
  robot_wpt_count++;
}


//************************** Callback functions ****************************//

void recordCamPoseCallback(const geometry_msgs::TransformStamped msg)
{
// This is hand's pose from VRPN
    q_h[0] =msg.transform.rotation.x;
    q_h[1] =msg.transform.rotation.y;
    q_h[2] =msg.transform.rotation.z;
    q_h[3] =msg.transform.rotation.w;
  
    Eigen::Quaternionf quat_h(q_h[3], q_h[0], q_h[1], q_h[2]);

    Rot_h2w = quat_h.matrix(); // Return the hand2world rotation matrix;

    if (raw_input == 1)
    {
      Rot_c2h << -0.2113,  0.0064, -0.9774,
                  0.9772, -0.0199, -0.2114,
                 -0.0208, -0.9998, -0.0020; 
      ocam_hand << -0.0655203, -0.0244237, 0.0046857;
    }
    else
    {
//   Brain5 Calibration on Oct 30th
      Rot_c2h <<  0.0918,  0.0170,  0.9956,
                 -0.9921,  0.0869,  0.0900,
                 -0.0850, -0.9961,  0.0248; 
      ocam_hand << 0.0750, -0.0221, 0.0023;
    }

    Rot_c2w = Rot_h2w * Rot_c2h;
    Eigen::AngleAxisf aa_cam_world;
    aa_cam_world = Rot_c2w;
    Eigen::Quaternionf quat_cam(aa_cam_world);
    q_cam = quat_cam.coeffs(); // returned in [x, y, z, w] order.

    ohand_world(0) = msg.transform.translation.x;
    ohand_world(1) = msg.transform.translation.y;
    ohand_world(2) = msg.transform.translation.z;
    ocam_world = ohand_world + Rot_h2w * ocam_hand;
//    std::cout << ocam_world(0) << " " << ocam_world(1) << std::endl;
    flag_getCurrentPosition = 1;
}

void pidCallback(const geometry_msgs::Point msg)
{
  pid_state = msg.x;
  ori_pid_state = msg.y;
  straight_pid_state = msg.z;
}

int getNumberofLine(std::string filename)
{
  int number_of_lines = 0;
  ifstream file;
  std::string line;

  file.open(filename.c_str());
    
  if (file.is_open()){
    while(std::getline(file, line))
    {
      ++number_of_lines;
    }
    return number_of_lines;
  }
  else
  { return 0; }
}

void showEntryList()
{
  int index_target;
  for (index_target = 1; index_target < NUM_TARGET+1; index_target++)
  {
    Eigen::MatrixXf mapp = EntryList[index_target];
    int row;
    std::cout << "EntryList[" << index_target << "] = " << "\n";
    for (row = 0; row < mapp.rows(); row++)
    { std::cout << EntryList[index_target](row, 0) << " " << EntryList[index_target](row, 1) << "\n"; }
  }
}

void showWaypointList(int index_target, int index_entry)
{
  Eigen::MatrixXf mapp = WaypointList[index_target][index_entry];
  int row;
  std::cout << "WaypointList[Target = " << index_target << "][Entry = " << index_entry << "] = " << "\n";
  for (row = 0; row < mapp.rows(); row++)
  { std::cout << WaypointList[index_target][index_entry](row, 0) << " " << WaypointList[index_target][index_entry](row, 1) << " " << WaypointList[index_target][index_entry](row, 2) << " " << WaypointList[index_target][index_entry](row, 3) << "\n"; }
}

void calcOrder(Eigen::Vector3f cam, Eigen::Vector3f targ){
	cam_goal = cam;
	tgt = targ;
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
//	std::cout << "Rg_hat: " << endl << Rg_hat << endl;
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

#endif
