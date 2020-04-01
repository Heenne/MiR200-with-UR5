#include <ur5_controller_manager/ur5_controller_info.h>

UR5ControllerInfo::UR5ControllerInfo(ros::NodeHandle robot_nh, std::string robot_name)
{
    this->robot_name_ = robot_name;
    this->robot_nh_ = robot_nh;
    this->action_completed_confirmation_ = false;
}

void UR5ControllerInfo::connectPlanTrajectoryAction(std::string action_name)
{
    this->plan_trajectory_ac_ = std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::PlanTrajectoryAction>>(
            new actionlib::SimpleActionClient<mir_ur5_msgs::PlanTrajectoryAction>(this->robot_nh_, action_name, true));
    this->plan_trajectory_ac_->waitForServer();
}

void UR5ControllerInfo::connectExecuteTrajectoryAction(std::string action_name)
{
    this->execute_trajectory_ac_ = std::shared_ptr<actionlib::SimpleActionClient<mir_ur5_msgs::ExecuteTrajectoryAction>>(
            new actionlib::SimpleActionClient<mir_ur5_msgs::ExecuteTrajectoryAction>(this->robot_nh_, action_name, true));
    this->execute_trajectory_ac_->waitForServer();
}

void UR5ControllerInfo::startPlanTrajectoryAction(mir_ur5_msgs::PlanTrajectoryGoal trajectory_goal)
{
    this->action_completed_confirmation_ = false;
    this->plan_trajectory_ac_->sendGoal(trajectory_goal,
                                        boost::bind(&UR5ControllerInfo::planTrajectoryDoneCallback, this, _1, _2),
                                        actionlib::SimpleActionClient<mir_ur5_msgs::PlanTrajectoryAction>::SimpleActiveCallback(),
                                        actionlib::SimpleActionClient<mir_ur5_msgs::PlanTrajectoryAction>::SimpleFeedbackCallback());
}

void UR5ControllerInfo::startExecuteTrajectoryAction(mir_ur5_msgs::ExecuteTrajectoryGoal trajectory_goal)
{
    this->action_completed_confirmation_ = false;
    this->execute_trajectory_ac_->sendGoal(trajectory_goal,
                                           boost::bind(&UR5ControllerInfo::executeTrajectoryDoneCallback, this, _1, _2),
                                           actionlib::SimpleActionClient<mir_ur5_msgs::ExecuteTrajectoryAction>::SimpleActiveCallback(),
                                           actionlib::SimpleActionClient<mir_ur5_msgs::ExecuteTrajectoryAction>::SimpleFeedbackCallback());
}


std::string UR5ControllerInfo::getRobotName()
{
    return this->robot_name_;
}

bool UR5ControllerInfo::isActionCompleted()
{
    if(!this->action_completed_confirmation_)
        ROS_INFO_STREAM("false " << this->robot_name_);
    else
        ROS_INFO_STREAM("true " << this->robot_name_);
    return this->action_completed_confirmation_;
}







void UR5ControllerInfo::planTrajectoryDoneCallback(const actionlib::SimpleClientGoalState &state,
                                                   const mir_ur5_msgs::PlanTrajectoryResultConstPtr &result)
{
    ROS_INFO_STREAM("plan done");
    // if(result->succeeded)
    // {
    //     ROS_INFO_STREAM("Received positive Done Callback by" << this->robot_name_);
    ROS_INFO_STREAM(this->robot_name_ << "before: " << std::to_string(this->action_completed_confirmation_));
    this->action_completed_confirmation_ = true;
    ROS_INFO_STREAM(this->robot_name_ << "after: " << std::to_string(this->action_completed_confirmation_));
    // }
    // else
    // {
    //     ROS_INFO_STREAM("Received negative Done Callback by" << this->robot_name_);
    // }
}

void UR5ControllerInfo::executeTrajectoryDoneCallback(const actionlib::SimpleClientGoalState &state,
                                                      const mir_ur5_msgs::ExecuteTrajectoryResultConstPtr &result)
{
    ROS_INFO_STREAM("execute done");
    // if(result->succeeded)
    // {
    //     ROS_INFO_STREAM("Received positive Done Callback by" << this->robot_name_);
        this->action_completed_confirmation_ = true;
    // }
}