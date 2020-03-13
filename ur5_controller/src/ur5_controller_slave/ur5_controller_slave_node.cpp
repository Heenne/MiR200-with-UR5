#include "ros/ros.h"

#include <ur5_controller/ur5_controller_slave.h>

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"ur5_controller_slave_node");
    ros::NodeHandle robot_node_handle;

    ros::AsyncSpinner spinner(2);
    spinner.start();   

    UR5ControllerSlave ur5_controller_slave = UR5ControllerSlave(robot_node_handle);

    ros::waitForShutdown();
}