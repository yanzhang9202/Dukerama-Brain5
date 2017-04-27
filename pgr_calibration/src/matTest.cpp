#include "iostream"
#include "fstream"


int main(int argc, char** argv)
{

std::ofstream outfile("/home/dukerama/catkin_ws/src/pgr_calibration/include/matFile.mat");

outfile << 5 << " ";
outfile << 6;
outfile << "\n";
outfile << 7 << " ";
outfile << 8;



outfile.close();

}



