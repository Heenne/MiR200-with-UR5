#pragma once

#include "ros/ros.h"

#pragma region Structs

#pragma endregion

#pragma region Enums

namespace UR5ControllerState
{
    enum UR5ControllerSlaveState
    {
        idle = 0,
        planning,
        plan_found,
        executing_plan
    };
}

/**
 * @brief This enum contains the ids that the task_id member of the "RobotArmInstructions.action" can be
 * 
 */
namespace RobotArmInstructionsTaskIds
{
    enum RobotArmInstructionsTaskIds
    {

    };
}

/**
 * @brief This enum contains the ids that the result_id member of the "RobotArmInstructions.action" can be
 * 
 */
namespace RobotArmInstructionsResultIds
{
    enum RobotArmInstructionsResultIds
    {
        
    };
}

/**
 * @brief This enum contains the ids that the progress_id member of the "RobotArmInstructions.action" can be
 * 
 */
namespace RobotArmInstructionsProgressIds
{
    enum RobotArmInstructionsProgressIds
    {
        
    };
}
#pragma endregion