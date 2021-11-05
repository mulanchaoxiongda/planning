#include <math.h>
#include <algorithm>
#include <string.h>
#include <fstream>
#include <iostream>
#include <string>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <cassert>
#include <iomanip>
#include <memory>
#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>
#include <iostream>
#include <sys/time.h>

#include "PlanningAlgorithm.h"

PlanningAlgorithm::PlanningAlgorithm(
        RobotModel *p_robot_model, SaveData *p_savedata, GoalState goal_state)
{
    p_robot_model_ = p_robot_model;
    p_savedata_ = p_savedata;

    goal_state_ = goal_state;

    GetSensorInfo();
}

void PlanningAlgorithm::GetSensorInfo()
{
    memcpy(&sensor_info_, &p_robot_model_->motion_state_, sizeof(SensorInfo));
}

