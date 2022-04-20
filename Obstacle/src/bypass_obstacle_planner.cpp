#include <osqp/osqp.h>

#include "bypass_obstacle_planner.h"

BypassObstaclePlanner::BypassObstaclePlanner(SaveData* p_savedata) {
    p_savedata_ = p_savedata;

    curve_smoother_ptr_ = new QpSplineSmoother(p_savedata);

    robot_len_  = 0.9;
    robot_wid_  = 0.7;
    dis_margin_ = 0.3 + 0.2; // todo : 定位误差30cm

    map_resolution_ = 0.05; // debug ： map_ptr_->GetResolution();

    planner_state_ = PlannerState::start;
}

BypassObstaclePlanner::~BypassObstaclePlanner() {
    delete curve_smoother_ptr_;
}

bool BypassObstaclePlanner::SetCurvePoints(const deque<CurvePoint>& curve_points) {
    return curve_smoother_ptr_->SetCurvePoints(curve_points);
}

void BypassObstaclePlanner::SetRobotPose(const RobotPose& pos) {
    robot_pose_ = pos;

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
    
    // LoadOccupyMap();

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

    global_path_smoothed_.clear();
    local_path_.clear();

    planner_state_ = PlannerState::start;
}

void PiecewiseJerkPathOptimization::LoadOccupyMap(string pathname) {
    vector<vector<bool>> cost_map;
    txt_to_vectordouble(cost_map, pathname);

    int rows = cost_map.size(), cols = cost_map[0].size();
    occupy_map_.resize(rows, cols);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            occupy_map_(i, j) = cost_map[i][j];
        }
    }
}

void PiecewiseJerkPathOptimization::LoadOccupyMap() {
    ; // occupy_map_ = map_ptr_->GetOccupyMap();
}

void PiecewiseJerkPathOptimization::SaveLog() {
    int nums_local_path_points = local_path_.size();
    if (!nums_local_path_points) {
        cout << "[error] there is no local path generated!" << endl;
    }

    for (auto point : local_path_) {
        p_savedata_->file << " [local_path] "
                          << " x "               << point.x
                          << " y "               << point.y
                          << " theta "           << point.theta
                          << " times_smoothing " << planning_times_
                          << endl;
    }

    int nums_smooth_points = global_path_smoothed_.size();
    if (!nums_smooth_points) {
        cout << "[error] there are no smooth line points!" << endl;
    }

    for (auto point : global_path_smoothed_) {
        p_savedata_->file << " [global_path_smoothed_] "
                          << " x "               << point.x
                          << " y "               << point.y
                          << " theta "           << point.theta
                          << " kappa "           << point.kappa
                          << " s "               << point.s
                          << endl;
    }

    DisplayInfo();
}

void PiecewiseJerkPathOptimization::DisplayInfo() const {
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

    for (auto point : local_path_) {
          cout << "[bypass_obstacle]" << "  "
               << "local_path :" << " "
               << point.s << "   "
               << point.x << "   "
               << point.y << "   "
               << point.theta << "   "
               << point.kappa << endl;
    }
}

void PiecewiseJerkPathOptimization::txt_to_vectordouble(
        vector<vector<bool>>& res, string pathname)
{
    string string_;

    ifstream read_file;
    read_file.open(pathname, ios::in);

    if (read_file.fail()) {
        cout << "[error] failed to open : " << pathname << endl;
    } else {
        while (getline(read_file, string_)) {
            istringstream is(string_);

            double data_;
            vector<bool> temp;

            while (!is.eof()) {
                is >> data_;
                temp.push_back(data_);
            }

            res.push_back(temp);
            temp.clear();
            string_.clear();
        }
    }

    read_file.close();
}
