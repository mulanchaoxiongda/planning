#pragma once

#include <deque>
#include <vector>
#include <eigen3/Eigen/Eigen>

#include "InformationFormat.h"
#include "SaveData.h"

using namespace std;

struct RobotPose {
    double x;
    double y;

    double theta;
};

struct SmoothLinePoint {
    double x;
    double y;
    double theta;
    double kappa;

    double s; // s-t速度规划使用
};

struct LocalPathPoint {
    double x;
    double y;
    double theta;
    double kappa;

    double s;
};

typedef enum {
      plan_suc,
      run_out_of_time,
      no_path_find,
      unknown_err
} PlannerStatus;

typedef enum {
    init,
    splicing,
    waiting,
    finished
} PlannerState;

class BypassObstaclePlanner {
      public:
      BypassObstaclePlanner() {};
      virtual ~BypassObstaclePlanner() {};

      virtual PlannerStatus GetLocalPath(deque<LocalPathPoint>& local_path_) = 0;

      void SetRobotPose(const RobotPose& pos);

      private:
      RobotPose pose_;
};
