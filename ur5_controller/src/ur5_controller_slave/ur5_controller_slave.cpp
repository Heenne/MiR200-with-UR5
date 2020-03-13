#include <ur5_controller/ur5_controller_slave.h>

UR5ControllerSlave::UR5ControllerSlave(ros::NodeHandle robot_node_handle)
{
    this->robot_node_handle_ = robot_node_handle;

    //Reading all parameter
    this->readParameters();

    this->ur5_controller_state_ = UR5ControllerSlave::UR5ControllerState::idle;

    ROS_INFO_STREAM("Created ur5 controller slave with following data:");
    ROS_INFO_STREAM("   - index: " << std::to_string(this->robot_index_));
    ROS_INFO_STREAM("   - robot_name: " << this->robot_name_);
    ROS_INFO_STREAM("   - robot_namespace: " << this->robot_namespace_);
    ROS_INFO_STREAM("   - robot_tf_prefix: " << this->robot_tf_prefix_);

    this->move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
    this->joint_model_group_ = std::shared_ptr<const moveit::core::JointModelGroup>(this->move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP));
    this->move_group_->setPlannerId("RRTConnect"); //Should be set as default planner in the configuration of the moveit package. But set explicitly to be sure.
    this->move_group_->setPlanningTime(1.0);
}

bool UR5ControllerSlave::setTargetPose(tf::Pose target_pose)
{
    this->target_pose_ = std::make_shared<tf::Pose>(target_pose);
    return true;
}

bool UR5ControllerSlave::planTrajectory()
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
        return true;
    }
    else
    {
        this->setControllerState(UR5ControllerState::idle);
        return false;
    }
}

bool UR5ControllerSlave::planTrajectory(tf::Pose target_pose)
{
    if(!this->setTargetPose(target_pose))
    {
        return false;
    }

    return this->planTrajectory();
}

//Getter/ Setter
void UR5ControllerSlave::setPlanningTimeoutAttempts(int planning_timeout_attempts)
{
    this->planning_attempts_timeout_ = planning_timeout_attempts;
}

int UR5ControllerSlave::getPlanningTimeoutAttempts()
{
    return this->planning_attempts_timeout_;
}

//Private methods
bool UR5ControllerSlave::readParameters()
{
    this->robot_node_handle_.param<int>("index", this->robot_index_, 0);
    this->robot_node_handle_.param<std::string>("robot_name", this->robot_name_, "");
    this->robot_node_handle_.param<std::string>("robot_namespace", this->robot_namespace_, "");
    this->robot_node_handle_.param<std::string>("robot_tf_prefix", this->robot_tf_prefix_, "");
    this->robot_node_handle_.param<int>("planning_attempts_timeout_", this->planning_attempts_timeout_, 10);
    this->robot_node_handle_.param<double>("planning_time", this->planning_time_, 1.0);
}

geometry_msgs::Pose UR5ControllerSlave::poseTFtoGeometryMsgs(tf::Pose pose)
{
    geometry_msgs::Pose geometry_pose;
    pose.setRotation(pose.getRotation().normalize()); //Normalize to be save
    tf::poseTFToMsg(pose, geometry_pose);
    return geometry_pose;
}

bool UR5ControllerSlave::setControllerState(UR5ControllerState target_ur5_controller_state)
{
    bool transition_result = false;
    switch (this->ur5_controller_state_)
    {
        case UR5ControllerState::idle:
        {
            transition_result = this->checkTransitionFromIdle(target_ur5_controller_state);
            break;
        }

        case UR5ControllerState::planning:
        {
            transition_result = this->checkTransitionFromPlanning(target_ur5_controller_state);
            break;
        }

        case UR5ControllerState::plan_found:
        {
            transition_result = this->checkTransitionFromPlanFound(target_ur5_controller_state);
            break;
        }

        case UR5ControllerState::executing_plan:
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

bool UR5ControllerSlave::checkTransitionFromIdle(UR5ControllerState target_ur_controller_state)
{
    switch(target_ur_controller_state)
    {
        case UR5ControllerState::idle:
        {
            return false;
            break;
        }
        case UR5ControllerState::planning:
        {
            return true;
            break;
        }
        case UR5ControllerState::plan_found:
        {
            return false;
            break;
        }
        case UR5ControllerState::executing_plan:
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

bool UR5ControllerSlave::checkTransitionFromPlanning(UR5ControllerState target_ur_controller_state)
{
    switch(target_ur_controller_state)
    {
        case UR5ControllerState::idle:
        {
            return true;
            break;
        }
        case UR5ControllerState::planning:
        {
            return false;
            break;
        }
        case UR5ControllerState::plan_found:
        {
            return true;
            break;
        }
        case UR5ControllerState::executing_plan:
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

bool UR5ControllerSlave::checkTransitionFromPlanFound(UR5ControllerState target_ur_controller_state)
{
    switch(target_ur_controller_state)
    {
        case UR5ControllerState::idle:
        {
            return true;
            break;
        }
        case UR5ControllerState::planning:
        {
            return false;
            break;
        }
        case UR5ControllerState::plan_found:
        {
            return false;
            break;
        }
        case UR5ControllerState::executing_plan:
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

bool UR5ControllerSlave::checkTransitionFromExecutingPlan(UR5ControllerState target_ur_controller_state)
{
    switch(target_ur_controller_state)
    {
        case UR5ControllerState::idle:
        {
            return true;
            break;
        }
        case UR5ControllerState::planning:
        {
            return false;
            break;
        }
        case UR5ControllerState::plan_found:
        {
            return false;
            break;
        }
        case UR5ControllerState::executing_plan:
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

