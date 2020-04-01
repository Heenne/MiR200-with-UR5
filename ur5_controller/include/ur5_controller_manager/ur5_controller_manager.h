#pragma once

#include "ros/ros.h"
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>


#include <ur5_controller_types/ur5_controller_types.h>
#include <ur5_controller_manager/ur5_controller_info.h>

#include <mir_ur5_msgs/PlanTrajectoryAction.h>
#include <mir_ur5_msgs/ExecuteTrajectoryAction.h>

class UR5ControllerManager
{
    public:
        UR5ControllerManager(ros::NodeHandle controller_manager_nh);

        void execute(const ros::TimerEvent &timer_event_info);
        
    private:
        enum class ExecuteStates
        {
            idle = 0,
            slaves_plan_trajectory,
            wait_for_planned_trajectory,
            execute_trajectory,
            wait_for_executed_trajectory
        };

        ros::NodeHandle ur5_controller_manager_nh_;

        std::vector<std::shared_ptr<UR5ControllerInfo>> ur5_controller_info_list_;

        ExecuteStates manager_state_;

        std::string general_robot_name_;
        int number_of_robots_;
        std::string plan_trajectory_action_name_;
        std::string execute_trajectory_action_name_;

        void loadParameter();
        bool checkIfAllSlavesConfirmed();        
};