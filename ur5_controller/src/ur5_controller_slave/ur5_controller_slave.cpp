#include <ur5_controller/ur5_controller_slave.h>

UR5ControllerSlave::UR5ControllerSlave(ros::NodeHandle robot_node_handle)
{
    this->robot_node_handle_ = robot_node_handle;

    //Reading all parameter
    this->loadParameters();

    this->ur5_controller_state_ = UR5ControllerState::UR5ControllerSlaveState::idle;

    ROS_INFO_STREAM("Created ur5 controller slave with following data:");
    ROS_INFO_STREAM("   - index: " << std::to_string(this->robot_index_));
    ROS_INFO_STREAM("   - robot_name: " << this->robot_name_);
    ROS_INFO_STREAM("   - robot_namespace: " << this->robot_namespace_);
    ROS_INFO_STREAM("   - robot_tf_prefix: " << this->robot_tf_prefix_);

    try
    {
        this->move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
        this->joint_model_group_ = std::shared_ptr<const moveit::core::JointModelGroup>(this->move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP));
        this->move_group_->setPlannerId("RRTConnect"); //Should be set as default planner in the configuration of the moveit package. But set explicitly to be sure.
        this->move_group_->setPlanningTime(1.0);
    }
    catch(const std::exception &e)
    {
        ROS_WARN("Error during the initialization of the MoveIt components. No robot arm movement possible!");
    }

    //Create action server for the planning of the trajectory, registering callbacks
    this->robot_arm_plan_trajectory_as_ = std::shared_ptr<actionlib::SimpleActionServer<mir_ur5_msgs::RobotArmPlanTrajectoryAction>>(
                                                new actionlib::SimpleActionServer<mir_ur5_msgs::RobotArmPlanTrajectoryAction>(this->robot_node_handle_, "UR5ControllerPlanTrajectory", false));
    this->robot_arm_plan_trajectory_as_->registerGoalCallback(boost::bind(&UR5ControllerSlave::robotArmPlanTrajectoryGoalCb, this));
    this->robot_arm_plan_trajectory_as_->registerPreemptCallback(boost::bind(&UR5ControllerSlave::robotArmPlanTrajectoryPreemptCb, this));

    this->robot_arm_execute_trajectory_as_ = std::shared_ptr<actionlib::SimpleActionServer<mir_ur5_msgs::RobotArmExecuteTrajectoryAction>>(
                                                new actionlib::SimpleActionServer<mir_ur5_msgs::RobotArmExecuteTrajectoryAction>(this->robot_node_handle_, "UR5ControllerExecuteTrajectory", false));
    this->robot_arm_execute_trajectory_as_->registerGoalCallback(boost::bind(&UR5ControllerSlave::robotArmExecuteTrajectoryGoalCb, this));
    this->robot_arm_execute_trajectory_as_->registerPreemptCallback(boost::bind(&UR5ControllerSlave::robotArmExecuteTrajectoryPreemptCb, this));

    //Start all action server
    this->robot_arm_plan_trajectory_as_->start();
    this->robot_arm_execute_trajectory_as_->start();
}

void UR5ControllerSlave::execute(const ros::TimerEvent &timer_event_info)
{

}

bool UR5ControllerSlave::planPTPTrajectory()
{
    if(!this->setControllerState(UR5ControllerState::planning))
    {
        return false; //State transition was invalid so planning failed
    }

    this->move_group_->setStartStateToCurrentState();
    this->move_group_->setPoseTarget(this->poseTFtoGeometryMsgs(*this->target_pose_));
    moveit::planning_interface::MoveGroupInterface::Plan movement_plan;
    
    bool planning_succeeded = false;
    int planning_attempts = 0;
    while(!planning_succeeded && planning_attempts < this->planning_attempts_timeout_)
    {
        moveit::planning_interface::MoveItErrorCode planning_result = this->move_group_->plan(movement_plan);
        switch (planning_result.val)
        {
            case moveit::planning_interface::MoveItErrorCode::SUCCESS:
            {
                planning_succeeded = true;
                break;
            }

            default:
            {
                break;
            }
        }

        planning_attempts = planning_attempts + 1;
    }

    if(planning_succeeded)
    {
        this->setControllerState(UR5ControllerState::plan_found);
        this->moveit_plan_ = std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>(movement_plan);
        this->moveit_plan_movement_type_ = UR5MovementTypeIds::PTP;
        ROS_INFO_STREAM("Planning trajectory by " << this->robot_name_ << " succeeded.");
        return true;
    }
    else
    {
        this->setControllerState(UR5ControllerState::idle);
        this->moveit_plan_ = nullptr; //Reset plan as none were found
        this->moveit_plan_movement_type_ = UR5MovementTypeIds::none;
        ROS_INFO_STREAM("Planning trajectory by " << this->robot_name_ << " failed.");
        return false;
    }
}

bool UR5ControllerSlave::planPTPTrajectory(tf::Pose target_pose)
{
    this->setTargetPose(target_pose);
    return this->planPTPTrajectory();
}

bool UR5ControllerSlave::planCartesianTrajectory()
{
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    moveit_msgs::MoveItErrorCodes planning_result;
    moveit_msgs::RobotTrajectory robot_trajectory;
    bool planning_succeeded = false;
    int planning_attempts = 0;

    if(!this->setControllerState(UR5ControllerState::planning))
    {
        return false; //State transition was invalid so planning failed
    }

    this->move_group_->setStartStateToCurrentState();
    std::vector<geometry_msgs::Pose> waypoint_list;
    waypoint_list.push_back(this->move_group_->getCurrentPose().pose); // Add first pose (the current state of the robot). Not needed but adviced in the tutorial
    waypoint_list.push_back(this->poseTFtoGeometryMsgs(*this->target_pose_)); // Add target position
    this->move_group_->setMaxVelocityScalingFactor(0.1); //Cartesian movement must be slower

    while(!planning_succeeded && planning_attempts < this->planning_attempts_timeout_)
    {
        double fraction = this->move_group_->computeCartesianPath(waypoint_list, eef_step, jump_threshold, robot_trajectory, true, &planning_result);

        switch(planning_result.val)
        {
            case moveit::planning_interface::MoveItErrorCode::SUCCESS:
            {
                planning_succeeded = true;
                break;
            }

            default:
            {
                
                break;
            }
        }

        planning_attempts = planning_attempts + 1;
    }

    if(planning_succeeded)
    {
        this->setControllerState(UR5ControllerState::plan_found);
        moveit::planning_interface::MoveGroupInterface::Plan moveit_plan_temp;
        moveit_plan_temp.trajectory_ = robot_trajectory;
        this->moveit_plan_ = std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>(moveit_plan_temp);
        this->moveit_plan_movement_type_ = UR5MovementTypeIds::Cartesian;
        ROS_INFO_STREAM("Planning trajectory by " << this->robot_name_ << " succeeded.");
        return true;
    }
    else
    {
        this->setControllerState(UR5ControllerState::idle);
        this->moveit_plan_ = nullptr;
        this->moveit_plan_movement_type_ = UR5MovementTypeIds::none;
        ROS_INFO_STREAM("Planning trajectory by " << this->robot_name_ << " failed.");
        return false;
    }
}

bool UR5ControllerSlave::planCartesianTrajectory(tf::Pose target_pose)
{
    this->setTargetPose(target_pose);
    return this->planCartesianTrajectory();
}

bool UR5ControllerSlave::planTrajectory(UR5MovementTypeIds::UR5MovementTypeIds ur5_movement_type_id)
{
    switch (ur5_movement_type_id)
    {
        case UR5MovementTypeIds::none:
        {   
            ROS_INFO_STREAM("No ur5_movement_type_id set.");
            return false;
            break;
        }
        case UR5MovementTypeIds::PTP:
        {
            return this->planPTPTrajectory();
            break;
        }
        case UR5MovementTypeIds::Cartesian:
        {
            return this->planCartesianTrajectory();
            break;
        }

        default:
        {
            ROS_INFO_STREAM("ur5_movement_type_id didn't match any predefined one.");
            return false;
            break;
        }
    }
}

bool UR5ControllerSlave::planTrajectory(UR5MovementTypeIds::UR5MovementTypeIds ur5_movement_type_id, tf::Pose target_pose)
{
    this->setTargetPose(target_pose);
    return this->planTrajectory(ur5_movement_type_id);
}

bool UR5ControllerSlave::executeTrajectory()
{
    moveit_msgs::MoveItErrorCodes error_code = this->move_group_->execute(*this->moveit_plan_);
    if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        return true;
    }
    else
    {
        //Do something
        return false;
    }
}


#pragma region Getter/Setter
tf::Pose UR5ControllerSlave::getTargetPose()
{
    return *this->target_pose_;
}

void UR5ControllerSlave::setTargetPose(tf::Pose target_pose)
{
    this->target_pose_ = std::make_shared<tf::Pose>(target_pose);
}

int UR5ControllerSlave::getPlanningTimeoutAttempts()
{
    return this->planning_attempts_timeout_;
}

void UR5ControllerSlave::setPlanningTimeoutAttempts(int planning_timeout_attempts)
{
    this->planning_attempts_timeout_ = planning_timeout_attempts;
}

std::string UR5ControllerSlave::getRobotName()
{
    return this->robot_name_;
}
#pragma endregion

#pragma region Callbacks
void UR5ControllerSlave::robotArmPlanTrajectoryGoalCb()
{
    bool planning_result = false;

    ROS_INFO_STREAM(this->robot_name_ << " received trajectory planning goal.");

    mir_ur5_msgs::RobotArmPlanTrajectoryGoalConstPtr goal_information = this->robot_arm_plan_trajectory_as_->acceptNewGoal();
    this->setTargetPose(this->poseGeometryMsgstoTF(goal_information->target_pose));

    
    if(this->planTrajectory(UR5MovementTypeIds::UR5MovementTypeIds(goal_information->movement_type_id)))
    {
        mir_ur5_msgs::RobotArmPlanTrajectoryResult result;
        result.succeeded = true;
        result.robot_id = this->robot_index_;
        result.result_id = 0;
        this->robot_arm_plan_trajectory_as_->setSucceeded(result);
    }
    else
    {
        // this->robot_arm_plan_trajectory_as_->setAborted();
    }
}

void UR5ControllerSlave::robotArmPlanTrajectoryPreemptCb()
{

}


void UR5ControllerSlave::robotArmExecuteTrajectoryGoalCb()
{
    bool planning_result = false;

    ROS_INFO_STREAM(this->robot_name_ << " received trajectory execution goal.");

    mir_ur5_msgs::RobotArmExecuteTrajectoryGoalConstPtr goal_information = this->robot_arm_execute_trajectory_as_->acceptNewGoal();
    
    ROS_INFO_STREAM("Executing robot trajectory");
    if(this->executeTrajectory())
    {
        this->robot_arm_plan_trajectory_as_->setSucceeded();
    }
    else
    {
        // this->robot_arm_plan_trajectory_as_->setAborted();
    }
}

void UR5ControllerSlave::robotArmExecuteTrajectoryPreemptCb()
{

}
#pragma endregion

#pragma region private Methods
bool UR5ControllerSlave::loadParameters()
{
    this->robot_node_handle_.param<int>("index", this->robot_index_, 0);
    this->robot_node_handle_.param<std::string>("robot_name", this->robot_name_, "");
    this->robot_node_handle_.param<std::string>("robot_namespace", this->robot_namespace_, "");
    this->robot_node_handle_.param<std::string>("robot_tf_prefix", this->robot_tf_prefix_, "");
    this->robot_node_handle_.param<int>("planning_attempts_timeout", this->planning_attempts_timeout_, 10);
    this->robot_node_handle_.param<double>("planning_time", this->planning_time_, 1.0);
    this->robot_node_handle_.param<int>("execution_attempts_timeout", this->execution_attempts_timeout_, 10);
}

geometry_msgs::Pose UR5ControllerSlave::poseTFtoGeometryMsgs(tf::Pose pose)
{
    geometry_msgs::Pose geometry_pose;
    pose.setRotation(pose.getRotation().normalize()); //Normalize to be save
    tf::poseTFToMsg(pose, geometry_pose);
    return geometry_pose;
}

tf::Pose UR5ControllerSlave::poseGeometryMsgstoTF(geometry_msgs::Pose pose)
{
    tf::Pose tf_pose;
    tf::poseMsgToTF(pose, tf_pose);
    tf_pose.setRotation(tf_pose.getRotation().normalize());
    return tf_pose;
}

bool UR5ControllerSlave::setControllerState(UR5ControllerState::UR5ControllerSlaveState target_ur5_controller_state)
{
    bool transition_result = false;
    switch (this->ur5_controller_state_)
    {
        case UR5ControllerState::UR5ControllerSlaveState::idle:
        {
            transition_result = this->checkTransitionFromIdle(target_ur5_controller_state);
            break;
        }

        case UR5ControllerState::UR5ControllerSlaveState::planning:
        {
            transition_result = this->checkTransitionFromPlanning(target_ur5_controller_state);
            break;
        }

        case UR5ControllerState::UR5ControllerSlaveState::plan_found:
        {
            transition_result = this->checkTransitionFromPlanFound(target_ur5_controller_state);
            break;
        }

        case UR5ControllerState::UR5ControllerSlaveState::executing_plan:
        {
            transition_result = this->checkTransitionFromExecutingPlan(target_ur5_controller_state);
            break;
        }
    }

    if(transition_result)
    {
        this->ur5_controller_state_ = target_ur5_controller_state;
        return true;
    }
    else
    {
        ROS_WARN_STREAM("Non-valid transition was about to happen. Check transition from: " << this->ur5_controller_state_ << ", to: " << target_ur5_controller_state);
        return false;
    }
    
}

bool UR5ControllerSlave::checkTransitionFromIdle(UR5ControllerState::UR5ControllerSlaveState target_ur_controller_state)
{
    switch(target_ur_controller_state)
    {
        case UR5ControllerState::UR5ControllerSlaveState::idle:
        {
            return false;
            break;
        }
        case UR5ControllerState::UR5ControllerSlaveState::planning:
        {
            return true;
            break;
        }
        case UR5ControllerState::UR5ControllerSlaveState::plan_found:
        {
            return false;
            break;
        }
        case UR5ControllerState::UR5ControllerSlaveState::executing_plan:
        {
            return false;
            break;
        }
        default:
        {
            return false;
            break;
        }
    }
}

bool UR5ControllerSlave::checkTransitionFromPlanning(UR5ControllerState::UR5ControllerSlaveState target_ur_controller_state)
{
    switch(target_ur_controller_state)
    {
        case UR5ControllerState::UR5ControllerSlaveState::idle:
        {
            return true;
            break;
        }
        case UR5ControllerState::UR5ControllerSlaveState::planning:
        {
            return false;
            break;
        }
        case UR5ControllerState::UR5ControllerSlaveState::plan_found:
        {
            return true;
            break;
        }
        case UR5ControllerState::UR5ControllerSlaveState::executing_plan:
        {
            return false;
            break;
        }
        default:
        {
            return false;
            break;
        }
    }
}

bool UR5ControllerSlave::checkTransitionFromPlanFound(UR5ControllerState::UR5ControllerSlaveState target_ur_controller_state)
{
    switch(target_ur_controller_state)
    {
        case UR5ControllerState::UR5ControllerSlaveState::idle:
        {
            return true;
            break;
        }
        case UR5ControllerState::UR5ControllerSlaveState::planning:
        {
            return false;
            break;
        }
        case UR5ControllerState::UR5ControllerSlaveState::plan_found:
        {
            return false;
            break;
        }
        case UR5ControllerState::UR5ControllerSlaveState::executing_plan:
        {
            return true;
            break;
        }
        default:
        {
            return false;
            break;
        }
    }
}

bool UR5ControllerSlave::checkTransitionFromExecutingPlan(UR5ControllerState::UR5ControllerSlaveState target_ur_controller_state)
{
    switch(target_ur_controller_state)
    {
        case UR5ControllerState::UR5ControllerSlaveState::idle:
        {
            return true;
            break;
        }
        case UR5ControllerState::UR5ControllerSlaveState::planning:
        {
            return false;
            break;
        }
        case UR5ControllerState::UR5ControllerSlaveState::plan_found:
        {
            return false;
            break;
        }
        case UR5ControllerState::UR5ControllerSlaveState::executing_plan:
        {
            return false;
            break;
        }
        default:
        {
            return false;
            break;
        }
    }
}
#pragma endregion