#include <osqp/osqp.h>

#include "bypass_obstacle_planner.h"

void BypassObstaclePlanner::SetRobotPose(const RobotPose& pos) {
      pose_ = pos;
}
