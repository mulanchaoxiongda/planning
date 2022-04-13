#include <osqp/osqp.h>

#include "bypass_obstacle_planner.h"

void BypassObstaclePlanner::SetAGVPose(const PoseInfo& pos) {
      pose_ = pos;
}
