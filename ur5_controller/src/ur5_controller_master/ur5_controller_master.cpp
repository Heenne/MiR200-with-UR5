#include <ur5_controller/ur5_controller_master.h>

UR5ControllerMaster::UR5ControllerMaster(ros::NodeHandle master_node_handle)
{
    this->loadParameter();

    this->master_node_handle_ = master_node_handle;
    ROS_INFO_STREAM(this->number_of_robots_);
    for(int counter = 0; counter < this->number_of_robots_; counter++)
    {
        std::string complete_robot_name = this->general_robot_name_ + std::to_string(counter);
        std::string robot_namespace = "/" + complete_robot_name + "_ns";
        ros::NodeHandle robot_node_handle(robot_namespace);
        this->ur5_controller_slave_list_.push_back(robot_node_handle);
        this->ur5_controller_slave_confirmation_list_.insert(std::pair<std::string, bool>(complete_robot_name, false));
        std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmPlanTrajectoryAction>> temp_planning_ac = std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmPlanTrajectoryAction>>(
            new actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmPlanTrajectoryAction>(robot_node_handle, "UR5ControllerPlanTrajectory", true));
        temp_planning_ac->waitForServer();
        this->slave_trajectory_planning_list_ac_.push_back(temp_planning_ac);
        std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmExecuteTrajectoryAction>> temp_execution_ac = std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmExecuteTrajectoryAction>>(
            new actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmExecuteTrajectoryAction>(robot_node_handle, "UR5ControllerExecuteTrajectory", true));
        temp_execution_ac->waitForServer();
        this->slave_trajectory_execution_list_ac_.push_back(temp_execution_ac);
    }
}

void UR5ControllerMaster::execute(const ros::TimerEvent &timer_event_info)
{
    switch (this->master_state_machine_)
    {
        case UR5ControllerMasterStateMachine::idle:
        {
            this->master_state_machine_ = UR5ControllerMasterStateMachine::slaves_plan_trajectory;
            break;
        }

        case UR5ControllerMasterStateMachine::slaves_plan_trajectory:
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
                mir_ur5_msgs::RobotArmPlanTrajectoryGoal trajectory_goal;
                trajectory_goal.movement_type_id = UR5MovementTypeIds::PTP;
                trajectory_goal.target_pose = target_pose1;
                this->sendSlaveControllerPlanTrajectoryGoal(slave_controller_ac, trajectory_goal);
            }
            this->master_state_machine_ = UR5ControllerMasterStateMachine::wait_for_planned_trajectory;
            break;
        }

        case UR5ControllerMasterStateMachine::wait_for_planned_trajectory:
        {
            ROS_INFO_STREAM("SPAM");
            if(this->checkIfAllSlavesConfirmed())
            {
                ROS_INFO_STREAM("All controller slaves confirmed the successful planning of a trajectory.");
                this->master_state_machine_ = UR5ControllerMasterStateMachine::execute_trajectory;
                ROS_INFO_STREAM("wait_for_planned_trajectory -> execute_trajectory");
            }
            break;
        }

        case UR5ControllerMasterStateMachine::execute_trajectory:
        {
            this->resetConfirmationList();
            for(auto &slave_controller_ac: this->slave_trajectory_execution_list_ac_)
            {
                mir_ur5_msgs::RobotArmExecuteTrajectoryGoal execution_goal;
                this->sendSlaveControllerExecuteTrajectoryGoal(slave_controller_ac, execution_goal);
            }
            this->master_state_machine_ = UR5ControllerMasterStateMachine::wait_for_executed_trajectory;
            break;
        }

        case UR5ControllerMasterStateMachine::wait_for_executed_trajectory:
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
void UR5ControllerMaster::slaveControllerPlanTrajectoryDoneCallback(const actionlib::SimpleClientGoalState &state,
                                                                    const mir_ur5_msgs::RobotArmPlanTrajectoryResultConstPtr &result)
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

void UR5ControllerMaster::slaveControllerExecuteTrajectoryDoneCallback(const actionlib::SimpleClientGoalState &state,
                                                                       const mir_ur5_msgs::RobotArmExecuteTrajectoryResultConstPtr &result)
{
    if(result->succeeded)
    {
        ROS_INFO_STREAM("Received positive Done Callback by robot" << std::to_string(result->robot_id));
        this->ur5_controller_slave_confirmation_list_[this->general_robot_name_ + std::to_string(result->robot_id)] = true;
    }
}
#pragma endregion

void UR5ControllerMaster::loadParameter()
{
    this->master_node_handle_.param<int>("number_of_robots", this->number_of_robots_, 2);
    this->master_node_handle_.param<std::string>("general_robot_name", this->general_robot_name_, "");
}

bool UR5ControllerMaster::checkIfAllSlavesConfirmed()
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

void UR5ControllerMaster::resetConfirmationList()
{
    for(auto confirmation_entry: this->ur5_controller_slave_confirmation_list_)
    {
        this->ur5_controller_slave_confirmation_list_[confirmation_entry.first] = false;
    }
}

void UR5ControllerMaster::sendSlaveControllerPlanTrajectoryGoal(std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmPlanTrajectoryAction>> &slave_controller_ac,
                                                                mir_ur5_msgs::RobotArmPlanTrajectoryGoal trajectory_goal)
{
    slave_controller_ac->sendGoal(trajectory_goal,
                                  boost::bind(&UR5ControllerMaster::slaveControllerPlanTrajectoryDoneCallback, this, _1, _2),
                                  actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmPlanTrajectoryAction>::SimpleActiveCallback(),
                                  actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmPlanTrajectoryAction>::SimpleFeedbackCallback());
}

void UR5ControllerMaster::sendSlaveControllerExecuteTrajectoryGoal(std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmExecuteTrajectoryAction>> &slave_controller_ac,
                                           mir_ur5_msgs::RobotArmExecuteTrajectoryGoal trajectory_goal)
{
    slave_controller_ac->sendGoal(trajectory_goal,
                                  boost::bind(&UR5ControllerMaster::slaveControllerExecuteTrajectoryDoneCallback, this, _1, _2),
                                  actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmExecuteTrajectoryAction>::SimpleActiveCallback(),
                                  actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmExecuteTrajectoryAction>::SimpleFeedbackCallback());
}