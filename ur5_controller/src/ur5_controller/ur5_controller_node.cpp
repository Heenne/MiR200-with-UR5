#include "ros/ros.h"

#include <ur5_controller/ur5_controller.h>

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"ur5_controller");
    ros::NodeHandle robot_controller_nh;

    ros::AsyncSpinner async_spinner(2);
    async_spinner.start();   

    UR5Controller ur5_controller = UR5Controller(robot_controller_nh);

    ros::Timer execute_timer = robot_controller_nh.createTimer(ros::Duration(0.01), &UR5Controller::execute, &ur5_controller);
    
    ROS_INFO_STREAM(ur5_controller.getRobotName() << " slave node is active and spinning.");
    
    ros::waitForShutdown();
}