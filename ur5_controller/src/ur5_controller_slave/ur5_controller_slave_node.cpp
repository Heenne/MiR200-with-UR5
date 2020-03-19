#include "ros/ros.h"

#include <ur5_controller/ur5_controller_slave.h>

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"ur5_controller_slave_node");
    ros::NodeHandle robot_node_handle;

    ros::AsyncSpinner spinner(2);
    spinner.start();   

    UR5ControllerSlave ur5_controller_slave = UR5ControllerSlave(robot_node_handle);

    ros::Timer controller_state_machine_timer = robot_node_handle.createTimer(ros::Duration(0.01), &UR5ControllerSlave::execute, &ur5_controller_slave);
    
    ROS_INFO_STREAM(ur5_controller_slave.getRobotName() << " slave node is active and spinning.");
    
    ros::waitForShutdown();
}