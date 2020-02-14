#include "ros/ros.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelState.h>

ros::Publisher gazebo_model_state_pub;
bool attach_object = false;

enum States
{
    plan1,
    exe1,
    plan2,
    exe2,
    attachObject,
    plan3,
    exe3,
    end
};

void jointStatesCallback(const sensor_msgs::JointStatePtr &jointStates)
{
    if(attachObject)
    {
        geometry_msgs::PoseStamped pose_in;
        pose_in.header.frame_id="wrist_3_link_ur5";
        pose_in.header.stamp = ros::Time::now();
        pose_in.pose.orientation.w = 1.0;
        pose_in.pose.position.x = 0.0;
        pose_in.pose.position.y = 0.0;
        pose_in.pose.position.z = 0.0;
        geometry_msgs::PoseStamped pose_out;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;
        
        try
        {
            if(!tfBuffer.canTransform("wrist_3_link_ur5", "map", ros::Time::now(), ros::Duration(0)))
            {
                ros::Duration sleep(3);
                sleep.sleep(); //Wait a second, maybe transform is then available
            }
            transformStamped = tfBuffer.lookupTransform("wrist_3_link_ur5", "map", ros::Time(0));
            tf2::doTransform(pose_in, pose_out, transformStamped);
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            return;
        }

        gazebo_msgs::ModelState model_state;
        model_state.model_name = "block";
        model_state.pose = pose_out.pose;
        model_state.reference_frame = "map";

        gazebo_model_state_pub.publish(model_state);
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"control_ur5_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();   
    
    States states = States::plan1;

    ros::Publisher cmdVelPublisher = nh.advertise<geometry_msgs::Twist>("mobile_base_controller/cmd_vel",1);
    ros::Subscriber joint_states_sub = nh.subscribe("joint_states", 1, jointStatesCallback);
    gazebo_model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);

    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    cmdVelPublisher.publish(twist);

    // BEGIN_TUTORIAL
    //
    // Setup
    // ^^^^^
    //
    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP = "ur5_arm";

    // The :move_group_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    
    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
                std::ostream_iterator<std::string>(std::cout, ", "));

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, M_PI_2, 0);
    quaternion.normalize();

    geometry_msgs::Pose target_pose1;
    geometry_msgs::Pose target_pose2;
    target_pose1.orientation = tf2::toMsg(quaternion);
    target_pose1.position.x = 1.0;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 0.15;

    moveit_msgs::RobotTrajectory trajectory;
    while(ros::ok())
    {
        switch(states)
        {
            case States::plan1:
            {
                // Planning to a Pose goal
                // ^^^^^^^^^^^^^^^^^^^^^^^
                // We can plan a motion for this group to a desired pose for the
                // end-effector.
                
                move_group.setPoseTarget(target_pose1);

                // Now, we call the planner to compute the plan and visualize it.
                // Note that we are just planning, not asking move_group
                // to actually move the robot.
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;

                // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
                // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
                move_group.setPlanningTime(10.0);
                
                bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

                if(success)
                {
                    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s");
                    states = States::exe1;
                    ROS_INFO("plan1 -> exe1");
                }
                else
                {
                    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s FAILED");
                    ros::shutdown();
                }

                
                break;
            }

            case States::exe1:
            {
                // Moving to a pose goal
                // ^^^^^^^^^^^^^^^^^^^^^
                //
                // Moving to a pose goal is similar to the step above
                // except we now use the move() function. Note that
                // the pose goal we had set earlier is still active
                // and so the robot will try to move to that goal. We will
                // not use that function in this tutorial since it is
                // a blocking function and requires a controller to be active
                // and report success on execution of a trajectory.

                /* Uncomment below line when working with a real robot */
                moveit_msgs::MoveItErrorCodes error_code = move_group.move();
                ROS_INFO("error_code: %i", error_code.val);

                ROS_INFO("exe1 -> plan2");
                states=States::plan2;
                break;
            }

            case States::plan2:
            {
                // Cartesian Paths
                // ^^^^^^^^^^^^^^^
                // You can plan a Cartesian path directly by specifying a list of waypoints
                // for the end-effector to go through. Note that we are starting
                // from the new start state above.  The initial pose (start state) does not
                // need to be added to the waypoint list but adding it can help with visualizations
                std::vector<geometry_msgs::Pose> waypoints;
                waypoints.push_back(target_pose1);

                target_pose2 = target_pose1;
                // target_pose2.orientation.w = 1.0;
                // target_pose2.orientation.x = 0.0;
                // target_pose2.orientation.y = 0.0;
                // target_pose2.orientation.z = 0.0;

                target_pose2.position.z -= 0.05;
                waypoints.push_back(target_pose2);  // down

                // target_pose2.position.y += 0.2;
                // waypoints.push_back(target_pose2);  // right

                // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
                // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
                // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
                move_group.setMaxVelocityScalingFactor(0.1);

                // We want the Cartesian path to be interpolated at a resolution of 1 cm
                // which is why we will specify 0.01 as the max step in Cartesian
                // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
                // Warning - disabling the jump threshold while operating real hardware can cause
                // large unpredictable motions of redundant joints and could be a safety issue
                
                const double jump_threshold = 0.0;
                const double eef_step = 0.01;
                moveit_msgs::MoveItErrorCodes error_code;
                double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true, &error_code);
                ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
                if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
                {
                    ROS_INFO("plan2 -> exe2");
                    states = States::exe2;
                }
                else
                {
                    ros::shutdown();
                }
                break;
            }

            case States::exe2:
            {
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                my_plan.trajectory_ = trajectory;
                move_group.execute(my_plan);

                // moveit_msgs::MoveItErrorCodes error_code = move_group.move();
                // ROS_INFO("error_code: %i", error_code.val);
                ROS_INFO("exe2 -> end");
                states = States::attachObject;
                break;
            }

            case States::attachObject:
            {
                attach_object = true;
                ROS_INFO("attachObject -> plan3");
                states = States::plan3;
                break;
            }
            
            case States::plan3:
            {
                std::vector<geometry_msgs::Pose> waypoints;
                waypoints.push_back(target_pose2);

                target_pose2.position.z += 0.05;
                waypoints.push_back(target_pose2);  // down

                move_group.setMaxVelocityScalingFactor(0.1);
                
                const double jump_threshold = 0.0;
                const double eef_step = 0.01;
                moveit_msgs::MoveItErrorCodes error_code;
                double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true, &error_code);
                ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
                if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
                {
                    ROS_INFO("plan3 -> exe3");
                    states = States::exe3;
                }
                else
                {
                    ros::shutdown();
                }
                break;
            }
            
            case States::exe3:
            {
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                my_plan.trajectory_ = trajectory;
                move_group.execute(my_plan);

                ROS_INFO("exe3 -> end");
                states = States::end;
                break;
            }
        }
    }
}