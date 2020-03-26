#include <ur5_controller_manager/ur5_controller_manager.h>

UR5ControllerManager::UR5ControllerManager(ros::NodeHandle master_controller_nh)
{
    this->loadParameter();

    this->master_controller_nh_ = master_controller_nh;
    
    for(int counter = 0; counter < this->number_of_robots_; counter++)
    {
        std::string complete_robot_name = this->general_robot_name_ + std::to_string(counter);
        std::string robot_namespace = "/" + complete_robot_name + "_ns";
        ros::NodeHandle robot_node_handle(robot_namespace);
        this->ur5_controller_slave_list_.push_back(robot_node_handle);
        this->ur5_controller_slave_confirmation_list_.insert(std::pair<std::string, bool>(complete_robot_name, false));
        std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::PlanTrajectoryAction>> temp_planning_ac = std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::PlanTrajectoryAction>>(
            new actionlib::SimpleActionClient<mir_ur5_msgs::PlanTrajectoryAction>(robot_node_handle, "UR5ControllerPlanTrajectory", true));
        temp_planning_ac->waitForServer();
        this->slave_trajectory_planning_list_ac_.push_back(temp_planning_ac);
        std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::ExecuteTrajectoryAction>> temp_execution_ac = std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::ExecuteTrajectoryAction>>(
            new actionlib::SimpleActionClient<mir_ur5_msgs::ExecuteTrajectoryAction>(robot_node_handle, "UR5ControllerExecuteTrajectory", true));
        temp_execution_ac->waitForServer();
        this->slave_trajectory_execution_list_ac_.push_back(temp_execution_ac);
    }
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
            tf::Quaternion quaternion;
            quaternion.setRPY(0, M_PI_2, 0);
            quaternion.normalize();

            geometry_msgs::Pose target_pose1;
            tf::quaternionTFToMsg(quaternion, target_pose1.orientation);
            target_pose1.position.x = 0.8;
            target_pose1.position.y = 0.0;
            target_pose1.position.z = 0.15;
            for(auto &slave_controller_ac: this->slave_trajectory_planning_list_ac_)
            {
                mir_ur5_msgs::PlanTrajectoryGoal trajectory_goal;
                trajectory_goal.movement_type_id = static_cast<int8_t>(MovementTypeIds::PTP);
                trajectory_goal.target_pose = target_pose1;
                this->startPlanTrajectoryAction(slave_controller_ac, trajectory_goal);
            }
            this->manager_state_ = ExecuteStates::wait_for_planned_trajectory;
            break;
        }

        case ExecuteStates::wait_for_planned_trajectory:
        {
            ROS_INFO_STREAM("SPAM");
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
            this->resetConfirmationList();
            for(auto &slave_controller_ac: this->slave_trajectory_execution_list_ac_)
            {
                mir_ur5_msgs::ExecuteTrajectoryGoal execution_goal;
                this->startExecuteTrajectoryAction(slave_controller_ac, execution_goal);
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
void UR5ControllerManager::planTrajectoryDoneCallback(const actionlib::SimpleClientGoalState &state,
                                                                    const mir_ur5_msgs::PlanTrajectoryResultConstPtr &result)
{
    if(result->succeeded)
    {
        ROS_INFO_STREAM("Received positive Done Callback by robot" << std::to_string(result->robot_id));
        ROS_INFO_STREAM(this->general_robot_name_ + std::to_string(result->robot_id));
        this->ur5_controller_slave_confirmation_list_[this->general_robot_name_ + std::to_string(result->robot_id)] = true;
        ROS_INFO_STREAM(std::to_string(this->ur5_controller_slave_confirmation_list_[this->general_robot_name_ + std::to_string(result->robot_id)]));
    }
    else
    {
        ROS_INFO_STREAM("Received negative Done Callback by robot" << std::to_string(result->robot_id));
    }
}

void UR5ControllerManager::executeTrajectoryDoneCallback(const actionlib::SimpleClientGoalState &state,
                                                                       const mir_ur5_msgs::ExecuteTrajectoryResultConstPtr &result)
{
    if(result->succeeded)
    {
        ROS_INFO_STREAM("Received positive Done Callback by robot" << std::to_string(result->robot_id));
        this->ur5_controller_slave_confirmation_list_[this->general_robot_name_ + std::to_string(result->robot_id)] = true;
    }
}
#pragma endregion

void UR5ControllerManager::loadParameter()
{
    this->master_controller_nh_.param<int>("number_of_robots", this->number_of_robots_, 2);
    this->master_controller_nh_.param<std::string>("general_robot_name", this->general_robot_name_, "");
}

bool UR5ControllerManager::checkIfAllSlavesConfirmed()
{
    for(auto robot_confirmation: this->ur5_controller_slave_confirmation_list_)
    {
        ROS_INFO_STREAM(std::to_string(robot_confirmation.second));
        if(!robot_confirmation.second)
        {
            return false;
        }
    }

    return true;
}

void UR5ControllerManager::resetConfirmationList()
{
    for(auto confirmation_entry: this->ur5_controller_slave_confirmation_list_)
    {
        this->ur5_controller_slave_confirmation_list_[confirmation_entry.first] = false;
    }
}

void UR5ControllerManager::startPlanTrajectoryAction(std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::PlanTrajectoryAction>> &slave_controller_ac,
                                                                mir_ur5_msgs::PlanTrajectoryGoal trajectory_goal)
{
    slave_controller_ac->sendGoal(trajectory_goal,
                                  boost::bind(&UR5ControllerManager::planTrajectoryDoneCallback, this, _1, _2),
                                  actionlib::SimpleActionClient<mir_ur5_msgs::PlanTrajectoryAction>::SimpleActiveCallback(),
                                  actionlib::SimpleActionClient<mir_ur5_msgs::PlanTrajectoryAction>::SimpleFeedbackCallback());
}

void UR5ControllerManager::startExecuteTrajectoryAction(std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::ExecuteTrajectoryAction>> &slave_controller_ac,
                                           mir_ur5_msgs::ExecuteTrajectoryGoal trajectory_goal)
{
    slave_controller_ac->sendGoal(trajectory_goal,
                                  boost::bind(&UR5ControllerManager::executeTrajectoryDoneCallback, this, _1, _2),
                                  actionlib::SimpleActionClient<mir_ur5_msgs::ExecuteTrajectoryAction>::SimpleActiveCallback(),
                                  actionlib::SimpleActionClient<mir_ur5_msgs::ExecuteTrajectoryAction>::SimpleFeedbackCallback());
}