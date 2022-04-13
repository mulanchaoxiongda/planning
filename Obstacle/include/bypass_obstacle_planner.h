#pragma once

#include <vector>
#include <eigen3/Eigen/Eigen>

#include "InformationFormat.h"
#include "SaveData.h"

using namespace std;

struct PoseInfo {
    double x;
    double y;

    double theta;
};

typedef enum {
      plan_suc,
      run_out_of_time,
      no_path_find,
      unknown_err
} PlanStatus;

class BypassObstaclePlanner {
      public:
      BypassObstaclePlanner() {};
      virtual ~BypassObstaclePlanner() {};

      void SetAGVPose(const PoseInfo& pos); // void SetCurrentPose(const NaviPose &pos);

      private:
      PoseInfo pose_;
};
