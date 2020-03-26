#pragma once

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>

#include <mir_ur5_msgs/PlanTrajectoryAction.h>
#include <mir_ur5_msgs/ExecuteTrajectoryAction.h>

class UR5ControllerInfo
{
    public:
        UR5ControllerInfo();
        
    private:
        std::string robot_name_;
        
        std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::PlanTrajectoryAction>> trajectory_planning_ac_;
        std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::ExecuteTrajectoryAction>> trajectory_execution_ac_;

        bool action_completed_confirmation_;
};