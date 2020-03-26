#pragma once

#include "ros/ros.h"

#pragma region Structs

#pragma endregion

#pragma region Enums

enum class UR5ControllerStates
{
    idle = 0,
    planning,
    plan_found,
    executing_plan
};

enum class MovementTypeIds
{
    none = 0,
    PTP = 1,
    Cartesian = 2
};
#pragma endregion