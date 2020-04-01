#pragma once

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>

#include <mir_ur5_msgs/PlanTrajectoryAction.h>
#include <mir_ur5_msgs/ExecuteTrajectoryAction.h>

class UR5ControllerInfo
{
    public:
        UR5ControllerInfo(ros::NodeHandle robot_nh, std::string robot_name);

        void connectPlanTrajectoryAction(std::string action_name);
        void connectExecuteTrajectoryAction(std::string action_name);

        void startPlanTrajectoryAction(mir_ur5_msgs::PlanTrajectoryGoal trajectory_goal);
        void startExecuteTrajectoryAction(mir_ur5_msgs::ExecuteTrajectoryGoal trajectory_goal);

        std::string getRobotName();

        bool isActionCompleted();
        
    private:
        std::string robot_name_;
        ros::NodeHandle robot_nh_;
        
        std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::PlanTrajectoryAction>> plan_trajectory_ac_;
        std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::ExecuteTrajectoryAction>> execute_trajectory_ac_;

        bool action_completed_confirmation_;


        //Callbacks
        void planTrajectoryDoneCallback(const actionlib::SimpleClientGoalState &state,
                                        const mir_ur5_msgs::PlanTrajectoryResultConstPtr &result);

        void executeTrajectoryDoneCallback(const actionlib::SimpleClientGoalState &state,
                                           const mir_ur5_msgs::ExecuteTrajectoryResultConstPtr &result);
};