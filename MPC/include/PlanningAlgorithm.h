#pragma once

#include <vector>

#include "RobotModel.h"
#include "InformationFormat.h"
#include "SaveData.h"

using namespace std;

class RobotModel;
class SaveData;


class PlanningAlgorithm
{
 public:
     PlanningAlgorithm(RobotModel *p_robot_model, SaveData *p_savedata,
                       GoalState goal_state);
     ~PlanningAlgorithm() {};

     virtual ControlCommand CalRefTrajectory() = 0;


 protected:
     void GetSensorInfo();

     virtual void ReadInGoalTraj() = 0;

     virtual void SmoothTrajcecory() = 0;

     SaveData *p_savedata_;
     RobotModel *p_robot_model_;

     GoalState goal_state_;
     SensorInfo sensor_info_;

     vector<TrajPoint> loacl_trajectory_points_;

     vector<TrajPoint> global_traj_points_;
     RefPoint global_ref_traj_point_;
};
