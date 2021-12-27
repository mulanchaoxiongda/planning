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
        RobotModel *p_robot_model, SaveData *p_savedata)
{
    p_robot_model_ = p_robot_model;
    p_savedata_ = p_savedata;

    GetSensorInfo();
}

void PlanningAlgorithm::GetSensorInfo()
{
    memcpy(&sensor_info_, &p_robot_model_->motion_state_, sizeof(SensorInfo));
}

// Todo: ESDF数据格式与接口
void PlanningAlgorithm::FindNearestObsDis(double pos_x, double pos_y)
{
    // pos_x, pos_y求取ESDF图索引值
    dis_obs_near_ = 10000.0; // 查询ESDF图
}

// Todo: ESDF数据格式与接口
void PlanningAlgorithm::FindNearestObsPos(double pos_x, double pos_y)
{
    // pos_x, pos_y求取ESDF图索引值
    dis_obs_near_ = 10000.0; // 查询ESDF图

    // ESDF梯度求yaw_obs

    obs_x_near_ = 0.0; // pos_x + dis_obs_near_ * cos(yaw_obs)
    obs_y_near_ = 0.0;
}
