#include "ros/ros.h"

#include <ur5_controller_manager/ur5_controller_manager.h>


int main(int argc, char* argv[])
{
    ros::init(argc,argv,"ur5_controller_manager");
    ros::NodeHandle master_controller_nh;

    UR5ControllerManager ur5_controller_master = UR5ControllerManager(master_controller_nh);

    ros::Timer execute_timer = master_controller_nh.createTimer(ros::Duration(0.01), &UR5ControllerManager::execute, &ur5_controller_master);
    
    ROS_INFO_STREAM("UR5ControllerManager is active and spinning.");
    
    ros::spin();
}