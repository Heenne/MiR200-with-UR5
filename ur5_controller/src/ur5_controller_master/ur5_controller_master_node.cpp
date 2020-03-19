#include "ros/ros.h"

#include <ur5_controller/ur5_controller_master.h>
#include <ur5_controller/ur5_controller_slave.h>


int main(int argc, char* argv[])
{
    ros::init(argc,argv,"ur5_controller_master_node");
    ros::NodeHandle master_node_handle;

    UR5ControllerMaster ur5_controller_master = UR5ControllerMaster(master_node_handle);

    ros::Timer controller_state_machine_timer = master_node_handle.createTimer(ros::Duration(0.01), &UR5ControllerMaster::execute, &ur5_controller_master);
    
    ROS_INFO_STREAM("UR5ControllerMaster is active and spinning.");
    
    ros::spin();
}