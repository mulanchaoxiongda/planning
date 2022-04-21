#include <osqp/osqp.h>

#include "bypass_obstacle_planner.h"

BypassObstaclePlanner::BypassObstaclePlanner(SaveData* p_savedata) {
    p_savedata_ = p_savedata;

    curve_smoother_ptr_ = new QpSplineSmoother(p_savedata);

    robot_len_  = 0.9; // todo : 读入参数
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

    if (!CalPathFesiableRegion()) {
        return PlannerStatus::planner_no_fesiable_region;
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

    if (smoother_status == SmootherStatus::fail_no_global_path ||
        smoother_status == SmootherStatus::fail_optimize) {
        return false;
    } else {
        return true;
    }
}

bool PiecewiseJerkPathOptimization::CalPathFesiableRegion() {
    vector<int> idx_range(2);
    idx_range = CalPathLenWithinMap();

    fesiable_region_.clear();

    double x, y, theta, kappa, s, lmin, lmax;
    for (int i = idx_range[0]; i <= idx_range[1]; ++i) {
        x = global_path_smoothed_[i].x;
        y = global_path_smoothed_[i].y;
        theta = global_path_smoothed_[i].theta;
        kappa = global_path_smoothed_[i].kappa;

        s = global_path_smoothed_[i].s;

        if (!CalPointFesiableRegion(lmin, lmax, global_path_smoothed_[i])) {
            cout << "[bypass_obatacle] there is no fesiable region！ "
                 << " At position : x = " << global_path_smoothed_[i].x << "cm,"
                 << " y = " << global_path_smoothed_[i].y << "cm ！" << endl;
            return false;
        }
        
        fesiable_region_.push_back({x, y, theta, kappa, s, lmin, lmax});
    }

    return true;
}

bool PiecewiseJerkPathOptimization::CalPointFesiableRegion(
        double& l_min, double& l_max, const SmoothLinePoint point_info) {
    double road_wid_margin = 0.001;
    double road_half_wid = robot_wid_ * 3.0 + road_wid_margin; // 虚拟车道宽度
    
    double step_l = 0.05; // step_l取栅格图分辨率
    double step_x = step_l * cos(point_info.theta + pi_half_);
    double step_y = step_l * sin(point_info.theta + pi_half_);

    int nums_l_occupy = (int)(2.0 * road_half_wid / step_l) + 1;
    double modified_road_half_wid =
            (double)((nums_l_occupy - 1) / 2) * step_l +  + road_wid_margin; // 修正车道宽度

    double sample_l = -modified_road_half_wid;
    double sample_x =
            point_info.x - modified_road_half_wid *
            cos(point_info.theta + pi_half_);
    double sample_y =
            point_info.y - modified_road_half_wid *
            sin(point_info.theta + pi_half_);

    vector<bool> l_occupy(nums_l_occupy, 1);

    double r_min = 1.0 / point_info.kappa;

    for (int i = 0; i <= nums_l_occupy; ++i) { // 在global_path_基准点切线的垂线段上以栅格地图分辨率采点，并判断占用情况
        if (point_info.kappa >= 0.0) { // s-l系下有奇异性的点视为障碍物占用点
            if (sample_l >= r_min) {
                continue;
            }
        } else {
            if (sample_l <= r_min) {
                continue;
            }
        }
        
        l_occupy[i] = IsPointOccupied(sample_x, sample_y); // todo : 暂定超出局部障碍栅格图范围的点为自由点
        
        sample_l += step_l;
        sample_x += step_x;
        sample_y += step_y;
    }

    int len_l_occupy = l_occupy.size();
    int left = -1, right = 0, left_widest = -1, right_widest = -1, interval_max = 0;
    bool occupy_state = 1; // 0 未占用 1 占用

    while (right < len_l_occupy) { // 搜索s-l系下最宽可通行路宽对应的边界[lmin, lmax]
        if (occupy_state) {
            if (l_occupy[right]) {
                ++right;
                continue;
            } else {
                occupy_state = 0;
                left = right;
                ++right;
                continue;
            }
        } else {
            if (l_occupy[right]) {
                occupy_state = 1;
                
                if (right - left > interval_max) {
                    right_widest = right - 1;
                    left_widest = left;
                    interval_max = right - left;
                }

                ++right;
                continue;
            } else {
                ++right;
                continue;
            }
        }
    }

    if (!occupy_state && !l_occupy[right - 1] && right - left > interval_max) {
        right_widest = right - 1;
        left_widest = left;
        interval_max = right - left;
    }

    l_min = -modified_road_half_wid + left_widest * step_l;
    l_max = -modified_road_half_wid + right_widest * step_l;

    double road_wid_min = robot_wid_ * 2.5; // todo : 最窄可通行路宽
    if (l_max - l_min >= road_wid_min) {
        return true;
    } else {
        return false;
    }
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
