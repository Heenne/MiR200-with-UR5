#pragma once

#include "ros/ros.h"
#include <tf/tf.h>

#include <memory.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/Pose.h>

class UR5ControllerSlave
{
public:
    enum UR5ControllerState
    {
        idle = 0,
        planning,
        plan_found,
        executing_plan
    };

    UR5ControllerSlave(ros::NodeHandle robot_node_handle);

    bool setTargetPose(tf::Pose target_pose);

    bool planTrajectory();
    bool planTrajectory(tf::Pose target_pose);

    void setPlanningTimeoutAttempts(int planning_timeout_attempts);
    int getPlanningTimeoutAttempts();
    
private:
    ros::NodeHandle robot_node_handle_;

    std::string PLANNING_GROUP = "ur5_arm";
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_; 
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::shared_ptr<const robot_state::JointModelGroup> joint_model_group_;

    int robot_index_;
    std::string robot_name_;
    std::string robot_namespace_;
    std::string robot_tf_prefix_;

    UR5ControllerSlave::UR5ControllerState ur5_controller_state_;

    int planning_attempts_timeout_; //Number of tries to find a plan for a trajectory before reporting error
    double planning_time_; //Time for getting a plan to the target position

    std::shared_ptr<tf::Pose> target_pose_; //Target pose of the robot arm is saved here
    std::shared_ptr<moveit_msgs::RobotTrajectory> planned_trajectory_; //Planned trajectory is saved here. Null if no trajectory is available or it was not planned.

    //Reads all parameters for the class from the rosparam server
    //return: true is reading succeeded
    bool readParameters();

    geometry_msgs::Pose poseTFtoGeometryMsgs(tf::Pose pose);

    bool setControllerState(UR5ControllerState target_ur5_controller_state);
    bool checkTransitionFromIdle(UR5ControllerState target_ur_controller_state);
    bool checkTransitionFromPlanning(UR5ControllerState target_ur_controller_state);
    bool checkTransitionFromPlanFound(UR5ControllerState target_ur_controller_state);
    bool checkTransitionFromExecutingPlan(UR5ControllerState target_ur_controller_state);
};