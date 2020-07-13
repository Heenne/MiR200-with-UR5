#include <ur5_controller_manager/ur5_controller_manager.h>

UR5ControllerManager::UR5ControllerManager(ros::NodeHandle ur5_controller_manager_nh)
{
    this->loadParameter();

    this->ur5_controller_manager_nh_ = ur5_controller_manager_nh;

    for(int counter = 0; counter < this->number_of_robots_; counter++)
    {
        std::string complete_robot_name = this->general_robot_name_ + std::to_string(counter);
        std::string robot_namespace = "/" + complete_robot_name + "_ns";
        ros::NodeHandle robot_node_handle(robot_namespace);
        std::shared_ptr<UR5ControllerInfo> ur5_controller_info = std::make_shared<UR5ControllerInfo>(UR5ControllerInfo(robot_node_handle, complete_robot_name));
        ur5_controller_info->connectPlanTrajectoryAction(this->plan_trajectory_action_name_);
        ur5_controller_info->connectExecuteTrajectoryAction(this->execute_trajectory_action_name_);
        this->ur5_controller_info_list_.push_back(ur5_controller_info);
    }

    tf::Quaternion quaternion1;
    geometry_msgs::Pose target_pose1;
    quaternion1.setRPY(0, M_PI_2, 0);
    quaternion1.normalize();
    tf::quaternionTFToMsg(quaternion1, target_pose1.orientation);
    target_pose1.position.x = 0.694;
    target_pose1.position.y = -0.05;
    target_pose1.position.z = 0.20;

    tf::Quaternion quaternion2;
    geometry_msgs::Pose target_pose2;
    quaternion2.setRPY(0, M_PI_2, 0);
    quaternion2.normalize();
    tf::quaternionTFToMsg(quaternion2, target_pose2.orientation);
    target_pose2.position.x = 0.368;
    target_pose2.position.y = -0.586;
    target_pose2.position.z = 0.20;

    tf::Quaternion quaternion3;
    geometry_msgs::Pose target_pose3;
    quaternion3.setRPY(0, M_PI_2, 0);
    quaternion3.normalize();
    tf::quaternionTFToMsg(quaternion3, target_pose3.orientation);
    target_pose3.position.x = 0.659;
    target_pose3.position.y = -0.544;
    target_pose3.position.z = 0.20;

    this->grip_point_list_.push_back(target_pose1);
    this->grip_point_list_.push_back(target_pose2);
    this->grip_point_list_.push_back(target_pose3);

}

void UR5ControllerManager::execute(const ros::TimerEvent &timer_event_info)
{
    switch (this->manager_state_)
    {
        case ExecuteStates::idle:
        {
            this->manager_state_ = ExecuteStates::slaves_plan_trajectory;
            break;
        }

        case ExecuteStates::slaves_plan_trajectory:
        {
            // tf::Quaternion quaternion;
            // quaternion.setRPY(0, M_PI_2, 0);
            // quaternion.normalize();
            // geometry_msgs::Pose target_pose1;
            // tf::quaternionTFToMsg(quaternion, target_pose1.orientation);
            // target_pose1.position.x = 0.8;
            // target_pose1.position.y = 0.0;
            // target_pose1.position.z = 0.15;
            int counter = 0;
            for(auto ur5_controller_info: this->ur5_controller_info_list_)
            {
                ROS_INFO_STREAM("This needs to happen twice.");
                mir_ur5_msgs::PlanTrajectoryGoal trajectory_goal;
                trajectory_goal.movement_type_id = static_cast<int8_t>(MovementTypeIds::PTP);
                trajectory_goal.target_pose = this->grip_point_list_[counter];
                ur5_controller_info->startPlanTrajectoryAction(trajectory_goal);
                counter += 1;
            }
            ROS_INFO_STREAM("slaves_plan_trajectory -> wait_for_planned_trajectory");
            this->manager_state_ = ExecuteStates::wait_for_planned_trajectory;
            break;
        }

        case ExecuteStates::wait_for_planned_trajectory:
        {
            if(this->checkIfAllSlavesConfirmed())
            {
                ROS_INFO_STREAM("All controller slaves confirmed the successful planning of a trajectory.");
                this->manager_state_ = ExecuteStates::execute_trajectory;
                ROS_INFO_STREAM("wait_for_planned_trajectory -> execute_trajectory");
            }
            break;
        }

        case ExecuteStates::execute_trajectory:
        {
            for(auto ur5_controller_info: this->ur5_controller_info_list_)
            {
                mir_ur5_msgs::ExecuteTrajectoryGoal execution_goal;
                ur5_controller_info->startExecuteTrajectoryAction(execution_goal);
            }
            this->manager_state_ = ExecuteStates::wait_for_executed_trajectory;
            break;
        }

        case ExecuteStates::wait_for_executed_trajectory:
        {
            
            ROS_INFO_STREAM("All executions finished.");
        }

        default:
        {
            
            break;
        }
    }
}

#pragma region Callbacks

#pragma endregion

void UR5ControllerManager::loadParameter()
{
    this->ur5_controller_manager_nh_.param<int>("number_of_robots", this->number_of_robots_, 2);
    this->ur5_controller_manager_nh_.param<std::string>("general_robot_name", this->general_robot_name_, "");
    this->ur5_controller_manager_nh_.param<std::string>("plan_trajectory_action_name", this->plan_trajectory_action_name_, "PlanTrajectoryActionNameNotFound");
    this->ur5_controller_manager_nh_.param<std::string>("execute_trajectory_action_name", this->execute_trajectory_action_name_, "ExecuteTrajectoryActionNameNotFound");
}

bool UR5ControllerManager::checkIfAllSlavesConfirmed()
{
    ROS_INFO_STREAM("Mark");
    for(auto ur5_controller_info: this->ur5_controller_info_list_)
    {
        ROS_INFO_STREAM("Robot name: " << ur5_controller_info->getRobotName());
        ROS_INFO_STREAM(std::to_string(ur5_controller_info->isActionCompleted()));
        if(!ur5_controller_info->isActionCompleted())
        {
            return false;
        }
    }
    ROS_INFO_STREAM("4");
    return true;
}
