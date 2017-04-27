#include "master.cpp"

using namespace std;

int main(int argc, char **argv){
  ros::init(argc, argv, "ic_master");
  ros::NodeHandle n;

  ic_master master(n);

  master.supervise();
  
  cout << "Job finished!" << endl << endl;
}
