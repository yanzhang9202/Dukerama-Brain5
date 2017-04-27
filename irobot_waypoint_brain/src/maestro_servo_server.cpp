#include <maestro_servo_driver/MaestroServerAction.h>
#include <actionlib/server/simple_action_server.h>
#include "maestro_serial_driver.cpp"

typedef actionlib::SimpleActionServer<maestro_servo_driver::MaestroServerAction> Server;

void execute(const maestro_servo_driver::MaestroServerGoalConstPtr& goal, Server* as)
{
    printf("Hello!\n");

    int fd = maestroConnect();

    maestroSetTarget(fd, 0, goal->servo_target);
    
    int count = 0;
    while(GetMovingState(fd,0)){
      if (count == 0) { printf("Moving...\n"); } count++;
    }// GetMovingState = true if any servo is still moving;

    int position = maestroGetPosition(fd, 0);
    printf("Current position is %d.\n", position/4);

    close(fd);

    as -> setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "maestro_servo_server");
  ros::NodeHandle n;
  Server server(n, "maestro_servo_server", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
