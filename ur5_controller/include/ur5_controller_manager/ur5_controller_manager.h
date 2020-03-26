#pragma once

#include "ros/ros.h"
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>


#include <ur5_controller_types/ur5_controller_types.h>
#include <mir_ur5_msgs/PlanTrajectoryAction.h>
#include <mir_ur5_msgs/ExecuteTrajectoryAction.h>

class UR5ControllerManager
{
    public:
        UR5ControllerManager(ros::NodeHandle master_controller_nh);

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

        ros::NodeHandle master_controller_nh_;

        // std::vector<ros::NodeHandle> ur5_controller_slave_list_;
        // std::vector<std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::PlanTrajectoryAction>>> slave_trajectory_planning_list_ac_;
        // std::vector<std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::ExecuteTrajectoryAction>>> slave_trajectory_execution_list_ac_;
        // std::map<std::string, bool> ur5_controller_slave_confirmation_list_;

        ExecuteStates manager_state_;

        std::string general_robot_name_;
        int number_of_robots_;

        #pragma region Callbacks
        void planTrajectoryDoneCallback(const actionlib::SimpleClientGoalState &state,
                                        const mir_ur5_msgs::PlanTrajectoryResultConstPtr &result);

        void executeTrajectoryDoneCallback(const actionlib::SimpleClientGoalState &state,
                                           const mir_ur5_msgs::ExecuteTrajectoryResultConstPtr &result);
        #pragma endregion

        void loadParameter();
        bool checkIfAllSlavesConfirmed();

        /**
         * @brief This method resets the list where all confirmations of all robot slaves are stored. 
         * 
         */
        void resetConfirmationList();

        void startPlanTrajectoryAction(std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::PlanTrajectoryAction>> &slave_controller_ac,
                                                    mir_ur5_msgs::PlanTrajectoryGoal trajectory_goal);

        void startExecuteTrajectoryAction(std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::ExecuteTrajectoryAction>> &slave_controller_ac,
                                                      mir_ur5_msgs::ExecuteTrajectoryGoal trajectory_goal);
};