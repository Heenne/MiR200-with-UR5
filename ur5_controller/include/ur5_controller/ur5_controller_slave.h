#pragma once

#include "ros/ros.h"
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>

#include <memory.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/Pose.h>

#include <ur5_controller_types/ur5_controller_types.h>
#include <mir_ur5_msgs/RobotArmInstructionAction.h>

class UR5ControllerSlave
{
public:
    #pragma region De-/Constructor
    UR5ControllerSlave(ros::NodeHandle robot_node_handle);
    #pragma endregion

    #pragma region Methods
    void execute(const ros::TimerEvent &timer_event_info);

    bool planTrajectory();
    bool planTrajectory(tf::Pose target_pose);
    #pragma endregion

    #pragma region Getter/Setter
    tf::Pose getTargetPose();
    void setTargetPose(tf::Pose target_pose);

    int getPlanningTimeoutAttempts();
    void setPlanningTimeoutAttempts(int planning_timeout_attempts);

    std::string getRobotName();
    #pragma endregion
    
private:
    #pragma region Member
    //ROS member
    ros::NodeHandle robot_node_handle_;

    std::shared_ptr<actionlib::SimpleActionServer<mir_ur5_msgs::RobotArmInstructionAction>> robot_arm_instruction_as_;

    //MoveIt member
    std::string PLANNING_GROUP = "ur5_arm";
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_; 
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::shared_ptr<const robot_state::JointModelGroup> joint_model_group_;

    //Robot State/Information member
    int robot_index_;
    std::string robot_name_;
    std::string robot_namespace_;
    std::string robot_tf_prefix_;

    UR5ControllerState::UR5ControllerSlaveState ur5_controller_state_;

    int planning_attempts_timeout_; //Number of tries to find a plan for a trajectory before reporting error
    double planning_time_; //Time for getting a plan to the target position

    //General member
    std::shared_ptr<tf::Pose> target_pose_; //Target pose of the robot arm is saved here
    std::shared_ptr<moveit_msgs::RobotTrajectory> planned_trajectory_; //Planned trajectory is saved here. Null if no trajectory is available or it was not planned.
    #pragma endregion

    #pragma region Callbacks
    void robotArmInstructionGoalCb();
    void robotArmInstructionDoneCb(const actionlib::SimpleClientGoalState &state,
                              const mir_ur5_msgs::RobotArmInstructionActionResultConstPtr &result);
    void robotArmInstructionFeedbackCb(const mir_ur5_msgs::RobotArmInstructionFeedback::ConstPtr& feedback);
    #pragma endregion


    #pragma region Methods
    /**
     * @brief Reads all parameters for the class from the rosparam server
     * 
     * @return true Reading parameters succeeded
     * @return false Reading parameters failed
     */
    bool readParameters();

    /**
     * @brief Uses the tf::poseTFToMsg method but without the need to use a reference so this can be used in a function within a single line
     * 
     * @param pose tf::Pose object that should be converted to a geometry_msgs::Pose object
     * @return Object of type geometry_msgs::Pose converted from tf::Pose 
     */
    geometry_msgs::Pose poseTFtoGeometryMsgs(tf::Pose pose);

    /**
     * @brief Uses the tf::poseTFToMsg method but without the need to use a reference so this can be used in a function within a single line
     * 
     * @param pose geometry_msgs::Pose object that should be converted to a tf::Pose object
     * @return Object of type tf::Pose converted from geometry_msgs::Pose 
     */
    tf::Pose poseGeometrytoTFMsgs(geometry_msgs::Pose pose);

    bool setControllerState(UR5ControllerState::UR5ControllerSlaveState target_ur5_controller_state);
    bool checkTransitionFromIdle(UR5ControllerState::UR5ControllerSlaveState target_ur_controller_state);
    bool checkTransitionFromPlanning(UR5ControllerState::UR5ControllerSlaveState target_ur_controller_state);
    bool checkTransitionFromPlanFound(UR5ControllerState::UR5ControllerSlaveState target_ur_controller_state);
    bool checkTransitionFromExecutingPlan(UR5ControllerState::UR5ControllerSlaveState target_ur_controller_state);
    #pragma endregion
};