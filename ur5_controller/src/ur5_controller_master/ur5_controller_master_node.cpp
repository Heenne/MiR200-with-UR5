#include "ros/ros.h"

#include <ur5_controller/ur5_controller_master.h>
#include <ur5_controller/ur5_controller_slave.h>


int main(int argc, char* argv[])
{
    ros::init(argc,argv,"ur5_controller_master_node");
    ros::NodeHandle global_node_handle_;
    ros::NodeHandle private_node_handle_("~");

    int number_of_robots;
    global_node_handle_.param<int>("number_of_robots", number_of_robots, 2);
    for(int counter = 0; counter < number_of_robots; counter++)
    {
        ros::NodeHandle robot_node_handle_("robot" + counter);
    }
    
    // int bla;
    // if(private_node_handle_.hasParam("arg0"))
    // {
    //     private_node_handle_.getParam("arg0", bla);
    //     ROS_INFO_STREAM(bla);
    // }
}