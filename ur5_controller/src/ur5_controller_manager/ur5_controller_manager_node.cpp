#include "ros/ros.h"

#include <ur5_controller_manager/ur5_controller_manager.h>


int main(int argc, char* argv[])
{
    ros::init(argc,argv,"ur5_controller_manager");
    ros::NodeHandle ur5_controller_manager_nh;

    UR5ControllerManager ur5_controller_manager = UR5ControllerManager(ur5_controller_manager_nh);

    ros::Timer execute_timer = ur5_controller_manager_nh.createTimer(ros::Duration(0.01), &UR5ControllerManager::execute, &ur5_controller_manager);
    
    ROS_INFO_STREAM("UR5ControllerManager is active and spinning.");
    
    ros::spin();
}