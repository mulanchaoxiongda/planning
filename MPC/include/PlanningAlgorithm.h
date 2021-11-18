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

     virtual ControlCommand CalRefTrajectory(vector<TrajPoint> &local_traj_points) = 0;


 protected:
     void GetSensorInfo();

     virtual void ReadInGoalTraj() = 0;

     SaveData *p_savedata_;
     RobotModel *p_robot_model_;

     GoalState goal_state_; // excepted parking state

     SensorInfo sensor_info_; // AGV motion state

     vector<TrajPoint> local_trajectory_points_;
};
