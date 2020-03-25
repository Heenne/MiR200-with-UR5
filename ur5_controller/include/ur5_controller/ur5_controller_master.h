#pragma once

#include "ros/ros.h"
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>


#include <ur5_controller_types/ur5_controller_types.h>
#include <mir_ur5_msgs/RobotArmPlanTrajectoryAction.h>
#include <mir_ur5_msgs/RobotArmExecuteTrajectoryAction.h>

class UR5ControllerMaster
{
    public:
        UR5ControllerMaster(ros::NodeHandle master_node_handle);

        void execute(const ros::TimerEvent &timer_event_info);
        
    private:
        ros::NodeHandle master_node_handle_;

        std::vector<ros::NodeHandle> ur5_controller_slave_list_;
        std::vector<std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmPlanTrajectoryAction>>> slave_trajectory_planning_list_ac_;
        std::vector<std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmExecuteTrajectoryAction>>> slave_trajectory_execution_list_ac_;
        std::map<std::string, bool> ur5_controller_slave_confirmation_list_;

        UR5ControllerMasterStateMachine::UR5ControllerMasterStateMachine master_state_machine_;

        std::string general_robot_name_;
        int number_of_robots_;

        #pragma region Callbacks
        void slaveControllerPlanTrajectoryDoneCallback(const actionlib::SimpleClientGoalState &state,
                                                        const mir_ur5_msgs::RobotArmPlanTrajectoryResultConstPtr &result);

        void slaveControllerExecuteTrajectoryDoneCallback(const actionlib::SimpleClientGoalState &state,
                                                        const mir_ur5_msgs::RobotArmExecuteTrajectoryResultConstPtr &result);
        #pragma endregion

        void loadParameter();
        bool checkIfAllSlavesConfirmed();

        /**
         * @brief This method resets the list where all confirmations of all robot slaves are stored. 
         * 
         */
        void resetConfirmationList();

        void sendSlaveControllerPlanTrajectoryGoal(std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmPlanTrajectoryAction>> &slave_controller_ac,
                                                    mir_ur5_msgs::RobotArmPlanTrajectoryGoal trajectory_goal);

        void sendSlaveControllerExecuteTrajectoryGoal(std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmExecuteTrajectoryAction>> &slave_controller_ac,
                                                      mir_ur5_msgs::RobotArmExecuteTrajectoryGoal trajectory_goal);
};