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
    
    if (!SmoothGlobalPath()) {
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
    base_points_.clear();
    fesiable_region_.clear();

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

void PiecewiseJerkPathOptimization::SmootherParaCfg() {
    ;
}

bool PiecewiseJerkPathOptimization::SmoothGlobalPath() {
    SmootherStatus smoother_status =
            curve_smoother_ptr_->GetSmoothLine(global_path_smoothed_);

    if (smoother_status == SmootherStatus::fail_no_global_path || smoother_status == SmootherStatus::fail_optimize) {
        return false;
    } else {
        return true;
    }
}

bool PiecewiseJerkPathOptimization::CalFesiableRegion() {
    vector<int> idx_range(2);
    idx_range = CalPathLenWithinMap();

    double sample_res = 0.05; // 采点间隔
    int nums_sample_point = (int) (len_ref_s_ / sample_res);

    // fesiable_region_;

    for (int i = 0; i < nums_sample_point; ++i) {
        ;
    }

    return true;
}

vector<int> PiecewiseJerkPathOptimization::CalPathLenWithinMap() {
    len_ref_s_ = 0.001;

    vector<double> robot_pos = {robot_pose_.x, robot_pose_.y};
    int idx_start = FindNearestPoint(robot_pos, global_path_smoothed_), idx_end;

    int len_global_path = global_path_smoothed_.size();
    for (int i = idx_start + 1; i < len_global_path; ++i) {
        if (!IsPointInMap(global_path_smoothed_[i].x, global_path_smoothed_[i].y,
                          robot_pose_.x, robot_pose_.y)) {
            idx_end = i - 1;

            break;
        }

        vector<double> rel_vec(2);
        rel_vec[0] = global_path_smoothed_[i].x - global_path_smoothed_[i -1].x;
        rel_vec[1] = global_path_smoothed_[i].y - global_path_smoothed_[i -1].y;
        len_ref_s_ += Norm(rel_vec);
    }

    return vector<int> {idx_start, idx_end};
}

double PiecewiseJerkPathOptimization::Norm(const vector<double> &x) const {
    double val = 0.0;

    for (auto elem: x) {
        val += elem * elem;
    }

    return sqrt(val);
}

int  PiecewiseJerkPathOptimization::FindNearestPoint(
                const vector<double>& position, const deque<SmoothLinePoint>& smooth_traj) {
    int num_points = smooth_traj.size();

    double dis, dis_pre;
    vector<double> rel_vec(2, 0.0);

    for (int i = 0; i < num_points; ++i) { // 忽略点在s-l系坐标的奇异性
        rel_vec = {
                position[0] - smooth_traj[i].x,
                position[1] - smooth_traj[i].y };
        dis = Norm(rel_vec);

        if (i >= 1 && dis >= dis_pre) {
            return i - 1;
        }

        dis_pre = dis;
    }

    return -1;
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
