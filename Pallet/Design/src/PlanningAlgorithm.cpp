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

    obs_x_near_.resize(3);
    obs_y_near_.resize(3);
}

void PlanningAlgorithm::GetSensorInfo()
{
    memcpy(&sensor_info_, &p_robot_model_->motion_state_, sizeof(SensorInfo));
}

// Todo: ESDF数据格式与接口
void PlanningAlgorithm::FindNearestObsDis(double pos_x, double pos_y)
{
    // pos_x, pos_y求取ESDF图索引值
    double dis_center2obs = 10000.0; // PS:查询ESDF图
    dis_obs_near_ = dis_center2obs - 0.05; // dis_center2obs - 地图分辨率 * 0.5
}

// Todo: ESDF数据格式与接口
void PlanningAlgorithm::FindNearestObsPos(double pos_x, double pos_y)
{
    // pos_x, pos_y求取ESDF图索引值
    double dis_center2obs = 10000.0; // 查询ESDF图

    double center_x = 0.0, center_y = 0.0;

    // ESDF梯度求yaw_obs
    // PS:遍历ind节点外三层24个栅格，选取dis最小的两个求指向角度并线性插值
    double yaw_obs = 0.0;

    // PS:添加两个障碍点，使得障碍物势场引导AGV沿着local_traj从非障碍物一侧，
    // 绕行避开障碍物
    double rel_dis_obs = 0.1;
    double rel_yaw_obs = 0.5;

    obs_x_near_.at(0) = center_x + dis_center2obs * cos(yaw_obs);
    obs_x_near_.at(1) =
            obs_x_near_.at(0) + rel_dis_obs * cos(yaw_obs + rel_yaw_obs);
    obs_x_near_.at(1) =
            obs_x_near_.at(0) + rel_dis_obs * cos(yaw_obs - rel_yaw_obs);

    obs_y_near_.at(0) = center_y + dis_center2obs * sin(yaw_obs);
    obs_y_near_.at(1) =
            obs_y_near_.at(0) + rel_dis_obs * sin(yaw_obs + rel_yaw_obs);
    obs_y_near_.at(1) =
            obs_y_near_.at(0) + rel_dis_obs * sin(yaw_obs - rel_yaw_obs);
}
