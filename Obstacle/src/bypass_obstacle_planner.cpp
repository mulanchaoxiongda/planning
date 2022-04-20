#include <osqp/osqp.h>

#include "bypass_obstacle_planner.h"

BypassObstaclePlanner::BypassObstaclePlanner(SaveData* p_savedata) {
    p_savedata_ = p_savedata;

    curve_smoother_ptr_ = new QpSplineSmoother(p_savedata);

    robot_len_  = 0.9;
    robot_wid_  = 0.7;
    dis_margin_ = 0.2;
}

BypassObstaclePlanner::~BypassObstaclePlanner() {
    delete curve_smoother_ptr_;
}

bool BypassObstaclePlanner::SetCurvePoints(const deque<CurvePoint>& curve_points) {
    return curve_smoother_ptr_->SetCurvePoints(curve_points);
}

void BypassObstaclePlanner::SetRobotPose(const RobotPose& pos) {
    pose_ = pos;

    curve_smoother_ptr_->SetRobotPose(pos);
}

PiecewiseJerkPathOptimization::PiecewiseJerkPathOptimization(
        SaveData* p_savedata) : BypassObstaclePlanner(p_savedata) {
    planning_times_ = 0;
}

PlannerStatus PiecewiseJerkPathOptimization::GetLocalPath(
        deque<LocalPathPoint>& local_path) {
    struct timeval t_start, t_end; // debug
    gettimeofday(&t_start,NULL);

    SmootherStatus smoother_status =
            curve_smoother_ptr_->GetSmoothLine(global_path_smoothed_);

    if (smoother_status == SmootherStatus::fail_no_global_path) {
        return PlannerStatus::smoother_no_global_path;
    } else if (smoother_status == SmootherStatus::fail_optimize) {
        return PlannerStatus::smoother_failed;
    }

    SaveLog();

    gettimeofday(&t_end, NULL);
    running_time_ =
                (t_end.tv_sec - t_start.tv_sec) +
                (double)(t_end.tv_usec - t_start.tv_usec) / 1000000.0;
    cout << "[bypass_obstacle] " << "piecewise jerk optimization function running time : "
                << running_time_ * 1000.0 << " ms." << endl;

    return PlannerStatus::planner_success;
}

void PiecewiseJerkPathOptimization::Reset() {
    curve_smoother_ptr_->Reset();

    planning_times_ = 0;
}

void PiecewiseJerkPathOptimization::SetCostMap(MatrixXd& cost_map) {
    ;
}

void PiecewiseJerkPathOptimization::GetCostMap() {
    ;
}

void PiecewiseJerkPathOptimization::SaveLog() {
    DisplayInfo();
}

void PiecewiseJerkPathOptimization::DisplayInfo() {
    return;

    for (auto point : global_path_smoothed_) {
          cout << "[bypass_obstacle]" << "  "
               << "global_path_smooth :" << " "
               << point.s << "   "
               << point.x << "   "
               << point.y << "   "
               << point.theta << "   "
               << point.kappa << endl;
    }
}
