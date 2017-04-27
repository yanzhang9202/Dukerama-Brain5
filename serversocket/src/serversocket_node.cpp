//#include "ServerSocket.h"
//#include "SocketException.h"
#include "ServerSocket.cpp"
#include "SocketException.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "Tank.h"
#include <sstream>
#include <string>
#include <iostream>
#include <string>

int main ( int argc, char **argv)
{
  ros::init(argc, argv, "snn_control_rb");
  ros::NodeHandle node;


  ros::Rate loop_rate(10);

  int count = 0;
  std::cout << "running....\n";

  try
    {
      // Create the socket
      ServerSocket server ( 3100 );

      while ( true )
	{

	  ServerSocket new_sock;
	  server.accept ( new_sock );

	  try
	    {
                  while(ros::ok())
                        {
                          std::string data;
		          new_sock >> data;
		          new_sock << data;
                          // std::cout << "running....\n"<<data<<"\nExiting.\n";
                          std::cout << "hello running....\n"<<data;  
                          ros::ServiceClient client=node.serviceClient<irobot_create_2_1::Tank>("tank");
                          irobot_create_2_1::Tank srv;
                          std::cout << "start\n"; 
                                 
                          std::string::size_type sz; 
                          double left=std::stod(data,&sz);
                          double right=std::stod(data.substr(sz)); 
                          
                          srv.request.left=left;
                          srv.request.right=right;
                          srv.request.clear=1;
                          if(client.call(srv))
                          {
                          ROS_INFO("Service call %d %d",srv.request.left,srv.request.right);
                          if(srv.response.success)
                          {ROS_INFO("Response");}
                          else
                          {ROS_INFO("No Response");}
                          }
                          else
                          {
                           ROS_INFO("Service NOT called");
                          }

                          ros::spinOnce();
                          loop_rate.sleep();
                          ++count;                                               
                        }  
                                   
		
	    }
	  catch ( SocketException& ) {}

	}
    }
  catch ( SocketException& e )
    {
      std::cout << "Exception was caught:" << e.description() << "\nExiting.\n";
    }

  return 0;
}

