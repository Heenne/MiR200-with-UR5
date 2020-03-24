#include <ur5_controller/ur5_controller_master.h>

UR5ControllerMaster::UR5ControllerMaster(ros::NodeHandle master_node_handle)
{
    this->loadParameter();

    this->master_node_handle_ = master_node_handle;
    ROS_INFO_STREAM(this->number_of_robots_);
    for(int counter = 0; counter < this->number_of_robots_; counter++)
    {
        std::string specific_robot_name = this->general_robot_name_;
        specific_robot_name.append(std::to_string(counter));
        std::string specific_robot_namespace = specific_robot_name.append("_ns");
        std::string root_namespace = "/";
        ros::NodeHandle robot_node_handle(root_namespace.append(specific_robot_namespace));
        this->ur5_controller_slave_list_.push_back(robot_node_handle);
        this->ur5_controller_slave_confirmation_list_.insert(std::pair<std::string, bool>(specific_robot_name, false));
        std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmPlanTrajectoryAction>> temp_ac = std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmPlanTrajectoryAction>>(
                                new actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmPlanTrajectoryAction>(robot_node_handle, "UR5ControllerPlanTrajectory", true));
        temp_ac->waitForServer();
        this->slave_trajectory_planning_list_ac_.push_back(temp_ac);
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
            if(this->checkIfAllSlavesConfirmed())
            {
                ROS_INFO_STREAM("All controller slaves confirmed the successful planning of a trajectory.");
                this->master_state_machine_ = UR5ControllerMasterStateMachine::idle;
            }
            break;
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
        this->ur5_controller_slave_confirmation_list_[this->general_robot_name_.append(std::to_string(result->robot_id))] = true;
    }
    else
    {
        ROS_INFO_STREAM("Received negative Done Callback by robot" << std::to_string(result->robot_id));
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
        if(!robot_confirmation.second)
        {
            return false;
        }
    }

    return true;
}

void UR5ControllerMaster::sendSlaveControllerPlanTrajectoryGoal(std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmPlanTrajectoryAction>> &slave_controller_ac,
                                                                mir_ur5_msgs::RobotArmPlanTrajectoryGoal trajectory_goal)
{
    slave_controller_ac->sendGoal(trajectory_goal,
                                    boost::bind(&UR5ControllerMaster::slaveControllerPlanTrajectoryDoneCallback, this, _1, _2),
                                    actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmPlanTrajectoryAction>::SimpleActiveCallback(),
                                    actionlib::SimpleActionClient<mir_ur5_msgs::RobotArmPlanTrajectoryAction>::SimpleFeedbackCallback());
}