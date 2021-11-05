#pragma once

#include "InformationFormat.h"
#include "SaveData.h"

class TrackingMPC;
class TrackingAlgorithm;
class PlanningAlgorithm;
class SaveData;

class RobotModel
{
 friend class TrackingMPC;
 friend class TrackingAlgorithm;
 friend class PlanningAlgorithm;

 public:
     RobotModel(RobotMotionStatePara motion_state, double step,
                SaveData *p_savedata);
     RobotMotionStatePara UpdateMotionState(ControlCommand control_command);

 private:
     SaveData *p_savedata_;
     RobotMotionStatePara motion_state_;

     double simulation_step_;
};