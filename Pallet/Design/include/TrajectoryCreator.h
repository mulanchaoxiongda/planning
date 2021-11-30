#pragma once

#include "InformationFormat.h"
#include "SaveData.h"

class SaveData;

class TrajectoryCreator
{
 public:
     TrajectoryCreator(SaveData *p_save_trajectory_reference, SaveData *p_save_result);
     void TrajCreator();

 private:
     SaveData *p_save_result_;
     SaveData *p_save_trajectory_reference_;
};