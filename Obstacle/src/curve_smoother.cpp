#include <osqp/osqp.h>
#include "curve_smoother.h"

CurveSmoother::CurveSmoother(SaveData* p_savedata) {
    p_savedata_ = p_savedata;
}

bool CurveSmoother::SetCurvePoints(const deque<CurvePoint>& curve_points) {
    if (curve_points.empty()) {
        cout << "[error] there is no global path received!" << endl;
        return false;
    }

    curve_points_ = curve_points;

    return true;
}

void CurveSmoother::SetRobotPose(const RobotPose& pose) {
    robot_pose_ = pose;
}

QpSplineSmoother::QpSplineSmoother(
        SaveData* p_savedata) : CurveSmoother(p_savedata) {
    len_init_ = 8.0;
    len_line_ = len_init_;
    len_increment_ = 2.0;
    len_fragment_ = 1.0;
    interval_sampling_ = 0.2;
    interval_sample_ = 0.01;
    ub_line_len_ = 10.0;
    nums_fragments_ = Double2Int(len_line_ / len_fragment_);
    nums_in_fragment_ = Double2Int(len_fragment_ / interval_sampling_);

    max_turn_angle_ = 20.0 / 57.3;

    weight_acc_x_  = 1.0;
    weight_jerk_x_ = 1.0;
    weight_pos_x_  = 2000.0;
    weight_acc_y_  = 1.0;
    weight_jerk_y_ = 1.0;
    weight_pos_y_  = 2000.0;
    max_pos_err_init_ = 0.04;
    max_pos_err_x_ = max_pos_err_init_;
    max_pos_err_y_ = max_pos_err_init_;
    pos_err_incre_ = 0.02;

    times_osqp_failed_ = 0;
    max_times_failed_ = 3;
    times_smoothing_ = 0;

    smoother_state_ = SmootherState::init;

    osqp_max_iteration_ = 200;
    osqp_eps_abs_ = 0.001;

    osqp_solver_exitflag_ = -1;
};

SmootherStatus QpSplineSmoother::GetSmoothLine(
        deque<SmoothLinePoint>& smooth_line_points) {
    struct timeval t_start, t_end; // debug
    gettimeofday(&t_start,NULL);

    if (curve_points_.empty()) { // 没有接收到下发的全局路径，报错并下发上帧局部路径
        times_osqp_failed_ = 0;
        cout << "[error] there is no global path received!" << endl;

        SaveLog();

        return SmootherStatus::fail_no_global_path;
    }

    if (smoother_state_ == SmootherState::finished) {
        return SmootherStatus::finish;
    }

    SmootherParaCfg();

    if (smoother_state_ == SmootherState::waiting) {
        SaveLog();
        return SmootherStatus::wait;
    }

    CalSamplingPoints();

    MatrixXd matrix_h =
            MatrixXd::Zero(12 * nums_fragments_, 12 * nums_fragments_);
    VectorXd matrix_f = MatrixXd::Zero(12 * nums_fragments_, 1);
    CalObjectiveFunc(matrix_h, matrix_f);

    MatrixXd matrix_a_equ =
            MatrixXd::Zero(6 + 6 * (nums_fragments_ - 1) + 6,
                           12 * nums_fragments_);
    MatrixXd matrix_b_equ =
            MatrixXd::Zero(6 + 6 * (nums_fragments_ - 1) + 6, 1);
    CalEqualityConstraint(matrix_a_equ, matrix_b_equ);

    MatrixXd matrix_a_inequ =
            MatrixXd::Zero((2 + 2) * (nums_fragments_ * nums_in_fragment_ - 1),
                           12 * nums_fragments_);
    MatrixXd matrix_b_inequ =
            MatrixXd::Zero((2 + 2) * (nums_fragments_ * nums_in_fragment_ - 1), 1);
    CalInequalityConstraint(matrix_a_inequ, matrix_b_inequ);

    VectorXd optimal_coefficient(nums_fragments_ * 12);

    MatrixXd matrix_a =
            MatrixXd::Zero(matrix_a_equ.rows() + matrix_a_inequ.rows(),
                           matrix_a_equ.cols());
    matrix_a << matrix_a_inequ, matrix_a_equ;
    
    VectorXd vector_lb =
            VectorXd::Zero(matrix_b_equ.rows() + matrix_b_inequ.rows());

    VectorXd matrix_bl_inequ(matrix_b_inequ.rows());
    for (int i = 0; i < matrix_b_inequ.rows(); ++i) {
        matrix_bl_inequ(i) = -10000000000.0;
    }
    vector_lb << matrix_bl_inequ, matrix_b_equ;

    VectorXd vector_ub =
            VectorXd::Zero(matrix_b_equ.rows() + matrix_b_inequ.rows());
    VectorXd matrix_bu_inequ(matrix_b_inequ.rows());
    matrix_bu_inequ = matrix_b_inequ;
    vector_ub << matrix_bu_inequ, matrix_b_equ;

    osqp_solver_exitflag_ =
            OptimizationSolver(optimal_coefficient, matrix_h, matrix_f, matrix_a,
                               vector_lb, vector_ub, osqp_max_iteration_,
                               osqp_eps_abs_);

    if (osqp_solver_exitflag_ != 0) { // 二次规划求解失败，则输出上一帧规划路径smooth_line_/ 上层下发的全局路径curve_points_，并上报warning / fail
        if (times_osqp_failed_ > max_times_failed_) { // osqp求解失败，报警并输出上帧局部路径；osqp连续3次求解失败，报错并输出上帧结果
            cout << "[error] global path is failed to be smoothed!" << endl;
            SaveLog();

            return SmootherStatus::fail_optimize;
        } else {
            cout << "[warning] global path is failed to be smoothed : " 
                 << times_osqp_failed_ << " times!"
                 << endl;
            SaveLog();

            return SmootherStatus::warning_optimize;
        }
    } else {
        CalSmoothTraj(optimal_coefficient, smooth_line_points);

        SaveLog();

        gettimeofday(&t_end, NULL);
        running_time_ =
                (t_end.tv_sec - t_start.tv_sec) +
                (double)(t_end.tv_usec - t_start.tv_usec) / 1000000.0;
        cout << "[smoother] " << "qp_spline function running time : "
                << running_time_ * 1000.0 << " ms." << endl;

        return SmootherStatus::success;
    }
}

void QpSplineSmoother::Reset() { // 每次规划任务前均需要重置配置
    len_line_ = len_init_;
    len_fragment_ = 1.0;
    interval_sampling_ = 0.2;
    interval_sample_ = 0.01;
    nums_fragments_ = Double2Int(len_line_ / len_fragment_);
    nums_in_fragment_ = Double2Int(len_fragment_ / interval_sampling_);

    max_pos_err_x_ = max_pos_err_init_;
    max_pos_err_y_ = max_pos_err_init_;
    
    times_osqp_failed_ = 0;
    times_smoothing_ = 0;

    smoother_state_ = SmootherState::init;
    osqp_solver_exitflag_ = -1;

    curve_points_.clear();
    sampling_points_.clear();
    smooth_line_.clear();
};

void QpSplineSmoother::CalSamplingPoints() {
    sampling_points_.clear();

    vector<double> accumulative_length;
    vector<double> pos_x;
    vector<double> pos_y;
    vector<double> pos_theta;

    int num_points = curve_points_.size();

    double s = 0.0;
    accumulative_length.push_back(s);
    pos_x.push_back(sample_start_point_.x);
    pos_y.push_back(sample_start_point_.y);
    pos_theta.push_back(sample_start_point_.theta);

    double min_margin = 0.001, len_line = len_line_ + min_margin;
    for (int i = 1; i < num_points; ++i) {
        double rel_dis =
                sqrt(pow(curve_points_.at(i).x - curve_points_.at(i - 1).x, 2.0) +
                     pow(curve_points_.at(i).y - curve_points_.at(i - 1).y, 2.0));
        s += rel_dis;

        accumulative_length.push_back(s);
        pos_x.push_back(curve_points_.at(i).x);
        pos_y.push_back(curve_points_.at(i).y);
        pos_theta.push_back(curve_points_.at(i).theta);

        if (s > len_line) {
            break;
        }
    }

    s = 0.0;
    int idx_init = 0;
    while (s <= len_line) {
        double x =
                FastInterpLinear(accumulative_length, pos_x, s, idx_init);
        double y =
                FastInterpLinear(accumulative_length, pos_y, s, idx_init);
        double theta =
                FastInterpLinear(accumulative_length, pos_theta, s, idx_init);

        sampling_points_.push_back({x, y, theta});

        s += interval_sampling_;
    }
}

void QpSplineSmoother::CalObjectiveFunc(
    MatrixXd& matrix_h, VectorXd& matrix_f) const {
    double s = len_fragment_;

    for (int i = 0; i < nums_fragments_; ++i) {
        MatrixXd matrix_h_acc = MatrixXd::Zero(6, 6);
        matrix_h_acc.block(2, 2, 4, 4) <<
                4.0 * pow(s, 1.0),   6.0 * pow(s, 2.0),             8.0 * pow(s, 3.0),           10.0 * pow(s, 4.0),
                6.0 * pow(s, 2.0),   12.0 * pow(s, 3.0),           18.0 * pow(s, 4.0),           24.0 * pow(s, 5.0),
                8.0 * pow(s, 3.0),   18.0 * pow(s, 4.0),  (144.0 / 5.0) * pow(s, 5.0),           40.0 * pow(s, 6.0),
                10.0 * pow(s, 4.0),  24.0 * pow(s, 5.0),           40.0 * pow(s, 6.0),  (400.0 / 7.0) * pow(s, 7.0);

        MatrixXd matrix_h_jerk = MatrixXd::Zero(6, 6);
        matrix_h_jerk.block(3, 3, 3, 3) <<
                36.0 * pow(s, 1.0),    72.0 * pow(s, 2.0),  120.0 * pow(s, 3.0),
                72.0 * pow(s, 2.0),   192.0 * pow(s, 3.0),  360.0 * pow(s, 4.0),
                120.0 * pow(s, 3.0),  360.0 * pow(s, 4.0),  720.0 * pow(s, 5.0);

        MatrixXd matrix_h_pos = MatrixXd::Zero(6, 6);
        matrix_h_pos <<
                pow(s, 0.0),  pow(s, 1.0),  pow(s, 2.0),  pow(s, 3.0),  pow(s, 4.0),  pow(s, 5.0),
                pow(s, 1.0),  pow(s, 2.0),  pow(s, 3.0),  pow(s, 4.0),  pow(s, 5.0),  pow(s, 6.0),
                pow(s, 2.0),  pow(s, 3.0),  pow(s, 4.0),  pow(s, 5.0),  pow(s, 6.0),  pow(s, 7.0),
                pow(s, 3.0),  pow(s, 4.0),  pow(s, 5.0),  pow(s, 6.0),  pow(s, 7.0),  pow(s, 8.0),
                pow(s, 4.0),  pow(s, 5.0),  pow(s, 6.0),  pow(s, 7.0),  pow(s, 8.0),  pow(s, 9.0),
                pow(s, 5.0),  pow(s, 6.0),  pow(s, 7.0),  pow(s, 8.0),  pow(s, 9.0),  pow(s, 10.0);

        MatrixXd matrix_hx = MatrixXd::Zero(6, 6),
                 matrix_hy = MatrixXd::Zero(6, 6);
        matrix_hx = 
                2.0 * (weight_acc_x_ * matrix_h_acc + 
                       weight_jerk_x_ * matrix_h_jerk + 
                       weight_pos_x_ * matrix_h_pos);
        matrix_hy = 
                2.0 * (weight_acc_y_ * matrix_h_acc + 
                       weight_jerk_y_ * matrix_h_jerk + 
                       weight_pos_y_ * matrix_h_pos);

        MatrixXd matrix_h_xy = MatrixXd::Zero(12, 12);
        matrix_h_xy.block(0, 0, 6, 6) = matrix_hx;
        matrix_h_xy.block(6, 6, 6, 6) = matrix_hy;

        int idx;
        idx = i * 12 + 0;
        matrix_h.block(idx, idx, 12, 12) = matrix_h_xy;

        MatrixXd matrix_fx = MatrixXd::Zero(6, 1),
                 matrix_fy = MatrixXd::Zero(6, 1),
                 matrix_f_xy = MatrixXd::Zero(6, 1);
        matrix_f_xy <<
                1, pow(s, 1.0), pow(s, 2.0), pow(s, 3.0), pow(s, 4.0), pow(s, 5.0);
        matrix_fx =
                -2.0 * sampling_points_.at((i + 1) * 5).x * weight_pos_x_ * matrix_f_xy;
        matrix_fy =
                -2.0 * sampling_points_.at((i + 1) * 5).y * weight_pos_y_ * matrix_f_xy;

        matrix_f.block(idx, 0, 6, 1) =  matrix_fx;
        matrix_f.block(idx + 6, 0, 6, 1) =  matrix_fy;
    }

    matrix_h = (matrix_h + matrix_h.transpose()) * 0.5;
}

void QpSplineSmoother::CalEqualityConstraint(
        MatrixXd& matrix_a_equ, MatrixXd& matrix_b_equ) const {
    double s = 0.0;

    MatrixXd matrix_ax_start = MatrixXd::Zero(3, 12);
    matrix_ax_start.block(0, 0, 3, 6) <<
            1.0,  s,     pow(s, 2.0),  pow(s, 3.0),        pow(s, 4.0),         pow(s, 5.0),
            0.0,  1.0,   2.0 * s,      3.0 * pow(s, 2.0),  4.0 * pow(s, 3.0),   5.0 * pow(s, 4.0),
            0.0,  0.0,   2.0,          6.0 * s,            12.0 * pow(s, 2.0),  20.0 * pow(s, 3.0);

    MatrixXd matrix_ay_start = MatrixXd::Zero(3, 12);
    matrix_ay_start.block(0, 6, 3, 6) <<
            1.0,  s,     pow(s, 2.0),  pow(s, 3.0),        pow(s, 4.0),         pow(s, 5.0),
            0.0,  1.0,   2.0 * s,      3.0 * pow(s, 2.0),  4.0 * pow(s, 3.0),   5.0 * pow(s, 4.0),
            0.0,  0.0,   2.0,          6.0 * s,            12.0 * pow(s, 2.0),  20.0 * pow(s, 3.0);

    matrix_a_equ.block(0, 0, 3, 12) << matrix_ax_start;
    matrix_a_equ.block(3, 0, 3, 12) << matrix_ay_start;

    double x_start, y_start, vx_start, vy_start, ax_start, ay_start;
    x_start = sampling_points_.at(0).x;
    y_start = sampling_points_.at(0).y;
    vx_start = cos(sampling_points_.at(0).theta);
    vy_start = sin(sampling_points_.at(0).theta);
    ax_start = 0.0;
    ay_start = 0.0;
    
    matrix_b_equ.block(0, 0, 6, 1) <<
            x_start, vx_start, ax_start, y_start, vy_start, ay_start;

    s = len_fragment_;

    MatrixXd matrix_ax_end = MatrixXd::Zero(3, 12);
    matrix_ax_end.block(0, 0, 3, 6) <<
            1.0,  s,    pow(s, 2.0),  pow(s, 3.0),        pow(s, 4.0),         pow(s, 5.0),
            0.0,  1.0,  2.0 * s,      3.0 * pow(s, 2.0),  4.0 * pow(s, 3.0),   5.0 * pow(s, 4.0),
            0.0,  0.0,  2.0,          6.0 * s,            12.0 * pow(s, 2.0),  20.0 * pow(s, 3.0);

    MatrixXd matrix_ay_end = MatrixXd::Zero(3, 12);
    matrix_ay_end.block(0, 6, 3, 6) << 
            1.0,  s,    pow(s, 2.0),  pow(s, 3.0),        pow(s, 4.0),         pow(s, 5.0),
            0.0,  1.0,  2.0 * s,      3.0 * pow(s, 2.0),  4.0 * pow(s, 3.0),   5.0 * pow(s, 4.0),
            0.0,  0.0,  2.0,          6.0 * s,            12.0 * pow(s, 2.0),  20.0 * pow(s, 3.0);

    int idx_row_end, idx_col_end;
    idx_row_end = 6 + 6 * (nums_fragments_ - 1) + 5;
    idx_col_end = 12 * nums_fragments_ - 1;

    matrix_a_equ.block(idx_row_end - 5, idx_col_end - 11, 3, 12) = matrix_ax_end;
    matrix_a_equ.block(idx_row_end - 2, idx_col_end - 11, 3, 12) = matrix_ay_end;

    double x_end, y_end, vx_end, vy_end, ax_end, ay_end;
    x_end = sampling_points_.back().x;
    y_end = sampling_points_.back().y;
    vx_end = cos(sampling_points_.back().theta);
    vy_end = sin(sampling_points_.back().theta);
    ax_end = 0.0;
    ay_end = 0.0;

    matrix_b_equ.block(idx_row_end - 5, 0, 6, 1) <<
            x_end, vx_end, ax_end, y_end, vy_end, ay_end;

    MatrixXd matrix_a_x_continuity = MatrixXd::Zero(3, 24),
             matrix_a_y_continuity = MatrixXd::Zero(3, 24);

    for (int i = 0; i < nums_fragments_ - 1; ++i) {
        s = len_fragment_;

        matrix_a_x_continuity.block(0, 0, 3, 6) <<
                1.0,  s,    pow(s, 2.0),  pow(s, 3.0),        pow(s, 4.0),         pow(s, 5.0),
                0.0,  1.0,  2.0 * s,      3.0 * pow(s, 2.0),  4.0 * pow(s, 3.0),   5.0 * pow(s, 4.0),
                0.0,  0.0,  2.0,          6.0 * s,            12.0 * pow(s, 2.0),  20.0 * pow(s, 3.0);
        matrix_a_y_continuity.block(0, 6, 3, 6) <<
                1.0,  s,    pow(s, 2.0),  pow(s, 3.0),        pow(s, 4.0),         pow(s, 5.0),
                0.0,  1.0,  2.0 * s,      3.0 * pow(s, 2.0),  4.0 * pow(s, 3.0),   5.0 * pow(s, 4.0),
                0.0,  0.0,  2.0,          6.0 * s,            12.0 * pow(s, 2.0),  20.0 * pow(s, 3.0);
        
        s = 0.0;
        
        matrix_a_x_continuity.block(0, 12, 3, 6) <<
                1.0,  s,    pow(s, 2.0),  pow(s, 3.0),        pow(s, 4.0),         pow(s, 5.0),
                0.0,  1.0,  2.0 * s,      3.0 * pow(s, 2.0),  4.0 * pow(s, 3.0),   5.0 * pow(s, 4.0),
                0.0,  0.0,  2.0,          6.0 * s,            12.0 * pow(s, 2.0),  20.0 * pow(s, 3.0);
        matrix_a_x_continuity.block(0, 12, 3, 6) =
                -1.0 * matrix_a_x_continuity.block(0, 12, 3, 6);
        matrix_a_y_continuity.block(0, 18, 3, 6) <<
                1.0,  s,    pow(s, 2.0),  pow(s, 3.0),        pow(s, 4.0),         pow(s, 5.0),
                0.0,  1.0,  2.0 * s,      3.0 * pow(s, 2.0),  4.0 * pow(s, 3.0),   5.0 * pow(s, 4.0),
                0.0,  0.0,  2.0,          6.0 * s,            12.0 * pow(s, 2.0),  20.0 * pow(s, 3.0);
        matrix_a_y_continuity.block(0, 18, 3, 6) =
                -1.0 * matrix_a_y_continuity.block(0, 18, 3, 6);

        int idx_row = 6 * (i + 1), idx_col = 12 * i;
        matrix_a_equ.block(idx_row, idx_col, 3, 24) = matrix_a_x_continuity;
        matrix_a_equ.block(idx_row + 3, idx_col, 3, 24) = matrix_a_y_continuity;
    }
}

void QpSplineSmoother::CalInequalityConstraint(
        MatrixXd& matrix_a_inequ, MatrixXd& matrix_b_inequ) const {
    MatrixXd matrix_ax_inequ = MatrixXd::Zero(2, 12),
             matrix_ay_inequ = MatrixXd::Zero(2, 12);

    double s = 0;
    
    for (int i = 0; i < nums_fragments_; ++i) {
        for (int j = 0; j < nums_in_fragment_; ++j) {
            if (i == nums_fragments_ - 1 && j == nums_in_fragment_ - 1) {
                break;
            }

            matrix_ax_inequ.setZero(2, 12);
            matrix_ay_inequ.setZero(2, 12);

            s = double(j + 1) / nums_in_fragment_ * len_fragment_;

            matrix_ax_inequ.block(0, 0, 1, 6) <<
                    1.0,  s,  pow(s, 2.0), pow(s, 3.0), pow(s, 4.0), pow(s, 5.0);
            matrix_ax_inequ.block(1, 0, 1, 6) = 
                    -1.0 * matrix_ax_inequ.block(0, 0, 1, 6);

            matrix_ay_inequ.block(0, 6, 1, 6) <<
                    1.0,  s,  pow(s, 2.0), pow(s, 3.0), pow(s, 4.0), pow(s, 5.0);
            matrix_ay_inequ.block(1, 6, 1, 6) =
                    -1.0 * matrix_ax_inequ.block(0, 0, 1, 6);

            int idx_row, idx_col;
            idx_row = nums_in_fragment_ * (2 + 2) * i + (2 + 2) * j;
            idx_col = 12 * i;

            matrix_a_inequ.block(idx_row, idx_col, 2, 12) = matrix_ax_inequ;
            matrix_a_inequ.block(idx_row + 2, idx_col, 2, 12) = matrix_ay_inequ;

            int idx_ref_point = nums_in_fragment_ * i + j + 1;
            double x_ref = sampling_points_.at(idx_ref_point).x;
            double y_ref = sampling_points_.at(idx_ref_point).y;

            MatrixXd matrix_bx_inequ = MatrixXd::Zero(2, 1),
                     matrix_by_inequ = MatrixXd::Zero(2, 1);
            matrix_bx_inequ << x_ref + max_pos_err_x_, -x_ref + max_pos_err_x_;
            matrix_by_inequ << y_ref + max_pos_err_y_, -y_ref + max_pos_err_y_;

            matrix_b_inequ.block(idx_row, 0, 2, 1) = matrix_bx_inequ;
            matrix_b_inequ.block(idx_row + 2, 0, 2, 1) = matrix_by_inequ;
        }
    }
}

c_int QpSplineSmoother::OptimizationSolver(
        VectorXd &optimal_solution, MatrixXd matrix_p, VectorXd vector_q, 
        MatrixXd matrix_Ac, VectorXd vector_l, VectorXd vector_u,
        c_int max_iteration, c_float eps_abs) {
    vector<c_float> p_x;
    c_int           p_nnz;
    vector<c_int>   p_i;
    vector<c_int>   p_p;

    MatrixToCCS(matrix_p, &p_x, p_nnz, &p_i, &p_p);

    vector<c_float> A_x;
    c_int           A_nnz;
    vector<c_int>   matrix_A_i;
    vector<c_int>   A_p;

    MatrixToCCS(matrix_Ac, &A_x, A_nnz, &matrix_A_i, &A_p);

    int length = vector_q.size();

    vector<c_float> q(length);

    for (int i = 0; i < length; i++) {
        q.at(i) = vector_q(i);
    }

    length = vector_l.size();

    vector<c_float> l(length);

    for (int i = 0; i < length; i++) {
        l.at(i) = vector_l(i);
    }

    length = vector_u.size();

    vector<c_float> u(length);

    for (int i = 0; i < length; i++) {
        u.at(i) = vector_u(i);
    }

    c_int m = matrix_Ac.rows(); // 约束数量
    c_int n = matrix_p.cols();  // 变量数量

    c_int exitflag = 0;

    OSQPWorkspace *work;
    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

    if (data) {
        data->n = n;
        data->m = m;

        data->P =
                csc_matrix(data->n, data->n, p_nnz,CopyData(p_x),
                           CopyData(p_i), CopyData(p_p));
        data->q = CopyData(q);

        data->A =
                csc_matrix(data->m, data->n, A_nnz, CopyData(A_x),
                           CopyData(matrix_A_i), CopyData(A_p));

        data->l = CopyData(l);
        data->u = CopyData(u);
    }

    if (settings) {
        osqp_set_default_settings(settings);

        settings->polish   = true;
        settings->verbose  = false;
        settings->max_iter = max_iteration;
        settings->eps_abs  = eps_abs;
        settings->alpha    = 1.0;
    }

    exitflag = osqp_setup(&work, data, settings);

    osqp_solve(work);

    auto status = work->info->status_val;
    if (status < 0 || (status != 1 && status != 2)) {
        cout << "failed optimization status:" << status << endl;

        osqp_cleanup(work);

        if (data) {
            if (data->A) {
                c_free(data->A);
            }

            if (data->P) {
                c_free(data->P);
            }

            c_free(data);
         }

        c_free(settings);

        return -100;
    } else if (work->solution == nullptr) {
        cout << "The solution from OSQP is nullptr" << endl;

        osqp_cleanup(work);

        if (data) {
            if (data->A) {
                c_free(data->A);
            }

            if (data->P) {
                c_free(data->P);
            }

            c_free(data);
        }

        c_free(settings);

        return -100;
    }

    for (int i = 0; i < n; ++i) {
        optimal_solution(i) = work->solution->x[i];
    }

    osqp_cleanup(work);

    if (data) {
        if (data->A) {
            c_free(data->A);
        }
        if (data->P) {
            c_free(data->P);
        }

        c_free(data);
    }

    if (settings) {
        c_free(settings);
    }

    return exitflag;
}

void QpSplineSmoother::MatrixToCCS(
        MatrixXd matrix_a, vector<c_float> *sm_x, c_int &sm_nnz,
        vector<c_int> *sm_i, vector<c_int> *sm_p) {
    sm_p->emplace_back(0);

    int num_cols = matrix_a.cols(), num_rows = matrix_a.rows(), nz = 0;

    if (num_cols == num_rows) { // 方阵且对称，取上三角矩阵元素
        for( int j = 0; j < num_cols; j++) {
            for (int i = 0; i <= j; i++) {
                if(fabs(matrix_a(i,j))>0.0000001) {
                    sm_x->emplace_back(matrix_a(i,j));
                    sm_i->emplace_back(i);

                    nz++;
                }
            }

            sm_p->emplace_back(nz);
        }
    } else if (num_cols < num_rows) { // 非方阵，取全部矩阵元素
        for( int j = 0; j < num_cols; j++) {
            for (int i = 0; i < num_rows; i++) {
                if(fabs(matrix_a(i,j))>0.0000001) {
                    sm_x->emplace_back(matrix_a(i,j));
                    sm_i->emplace_back(i);

                    nz++;
                }
            }

            sm_p->emplace_back(nz);
        }
    }

    sm_nnz = nz;
}

double QpSplineSmoother::InterpLinear(
        const vector<double>& x, const vector<double>& y, double x0) const {
    long unsigned int i = x.size() - 1;
    long unsigned int j;

    if (x0 < x.at(0)) {
        x0 = x.at(0) + 0.0001;
    }
    else if (x0 > x.at(i)) {
        x0 = x.at(i) - 0.0001;
    }

    for (j = 0; j < i; j++) {
        double temp = (x0 - x.at(j)) * (x0 - x.at(j + 1));

        if (temp <= 0.0) {
            break;
        }
    }

    double y_result =
            y.at(j) + (x0 - x.at(j)) * (y.at(j + 1) -
            y.at(j)) / (x.at(j + 1) - x.at(j));

    return y_result;
}

double QpSplineSmoother::FastInterpLinear(
        const vector<double>& x, const vector<double>& y,
        double x0, int& idx_init) const {
    long unsigned int i = x.size() - 1;
    long unsigned int j;

    if (x0 < x.at(idx_init)) {
        x0 = x.at(idx_init) + 0.0001;
    }
    else if (x0 > x.at(i)) {
        x0 = x.at(i) - 0.0001;
    }

    for (j = idx_init; j < i; j++) {
        double temp = (x0 - x.at(j)) * (x0 - x.at(j + 1));

        if (temp <= 0.0) {
            break;
        }
    }

    double y_result =
            y.at(j) + (x0 - x.at(j)) * (y.at(j + 1) -
            y.at(j)) / (x.at(j + 1) - x.at(j));

    idx_init = j;

    return y_result;
}

void QpSplineSmoother::Txt2Vector(
        deque<CurvePoint>& res, const string& pathname) {
    string string_;

    ifstream read_file;
    read_file.open(pathname, ios::in);

    if (read_file.fail()) {
        cout << "[error] failed to open : " << pathname << endl;
    } else {
        res.resize(0);

        while (getline(read_file, string_)) {
            istringstream is(string_);

            double data_;
            vector<double> temp;

            while (!is.eof()) {
                is >> data_;
                temp.push_back(data_);
            }

            res.push_back({temp[0], temp[1], temp[2]});
            temp.clear();
            string_.clear();
        }
    }

    read_file.close();
}

void QpSplineSmoother::CalSmoothTraj(
    const VectorXd& poly_coefficient, deque<SmoothLinePoint>& smooth_line) {
    double nums_coef = 12;
    vector<double> poly_coef(nums_coef, 0);
    double s = 0.0, x, y, theta, kappa, s_accumulative;
    double dx, ddx, dy, ddy;
    double len_margin =0.001, len_fragment = len_fragment_ + len_margin;
    
    s_accumulative = (smooth_line_.empty()) ? 0.0 : smooth_line_.back().s;

    for (int i = 0; i < nums_fragments_; ++i) {
        for (int j = 0; j < nums_coef; ++j) {
            poly_coef[j] = poly_coefficient[i * nums_coef + j];
        }

        if (i != 0 || smoother_state_ != SmootherState::init) {
            s = interval_sample_;
        }

        while (s < len_fragment) {
            x =
                    poly_coef[0] + poly_coef[1] * s + poly_coef[2] * pow(s, 2.0) +
                    poly_coef[3] * pow(s, 3.0) + poly_coef[4] * pow(s, 4.0) +
                    poly_coef[5] * pow(s, 5.0);
            y =
                    poly_coef[6] + poly_coef[7] * s + poly_coef[8] * pow(s, 2.0) +
                    poly_coef[9] * pow(s, 3.0) + poly_coef[10] * pow(s, 4.0) +
                    poly_coef[11] * pow(s, 5.0);
            dx =
                    poly_coef[1] + 2.0 * poly_coef[2] * pow(s, 1.0) +
                    3.0 * poly_coef[3] * pow(s, 2.0) +
                    4.0 * poly_coef[4] * pow(s, 3.0) +
                    5.0 * poly_coef[5] * pow(s, 4.0);
            dy =
                    poly_coef[7] + 2.0 * poly_coef[8] * pow(s, 1.0) +
                    3.0 * poly_coef[9] * pow(s, 2.0) +
                    4.0 * poly_coef[10] * pow(s, 3.0) +
                    5.0 * poly_coef[11] * pow(s, 4.0);
            ddx =
                    2.0 * poly_coef[2] + 6.0 * poly_coef[3] * pow(s, 1.0) +
                    12.0 * poly_coef[4] * pow(s, 2.0) +
                    20.0 * poly_coef[5] * pow(s, 3.0);
            ddy =
                    2.0 * poly_coef[8] + 6.0 * poly_coef[9] * pow(s, 1.0) +
                    12.0 * poly_coef[10] * pow(s, 2.0) +
                    20.0 * poly_coef[11] * pow(s, 3.0);
            
            theta = atan2(dy, dx);
            kappa =
                    fabs(dx * ddy - dy * ddx) / pow(pow(dx, 2.0) +
                    pow(dy, 2.0), 1.5);

            smooth_line_.push_back({x, y, theta, kappa, s_accumulative + s});

            s += interval_sample_;
        }

        s_accumulative += s;
    }

    ShearCutTraj();

    smooth_line = smooth_line_;
}

void QpSplineSmoother::ShearCutTraj() { // 剪切掉s-l系下，s值小于机器人当前位置的smooth_point点
    vector<double> robot_pos = {robot_pose_.x, robot_pose_.y};

    int idx_int = FindNearestPoint(robot_pos, smooth_line_);
    idx_int = idx_int > 1 ? (idx_int -1) : idx_int;

    for (int i = 0; i < idx_int; ++i) {
        smooth_line_.pop_front();
    }
}

double QpSplineSmoother::Norm(const vector<double> &x) const {
    double val = 0.0;

    for (auto elem: x) {
        val += elem * elem;
    }

    return sqrt(val);
}

double QpSplineSmoother::VecDotMultip(
        const vector<double> &x, const vector<double> &y) const {
    assert(x.size() == y.size());

    double sum = 0.0;

    for (size_t i = 0; i < x.size(); ++i) {
        sum += x[i] * y[i];
    }

    return sum;
}

void QpSplineSmoother::SaveLog() {
    int nums_curve_points = curve_points_.size();
    if (!nums_curve_points) {
        cout << "[error] there is no global path received!" << endl;
    }

    for (int i = 0; i < nums_curve_points; ++i) {
        p_savedata_->file << " [golbal_path_points] "
                          << " x "               << curve_points_[i].x
                          << " y "               << curve_points_[i].y
                          << " theta "           << curve_points_[i].theta
                          << " times_smoothing " << times_smoothing_
                          << endl;
    }

    int nums_sample_points = sampling_points_.size();
    if (!nums_sample_points) {
        cout << "[error] there are no sampling points!" << endl;
    }

    for (int i = 0; i < nums_sample_points; ++i) {
        p_savedata_->file << " [sampling_points] "
                          << " x "               << sampling_points_[i].x
                          << " y "               << sampling_points_[i].y
                          << " theta "           << sampling_points_[i].theta
                          << " times_smoothing " << times_smoothing_
                          << endl;
    }

    int nums_smooth_points = smooth_line_.size();
    if (!nums_smooth_points) {
        cout << "[error] there are no smooth line points!" << endl;
    }

    for (int i = 0; i < nums_smooth_points; ++i) {
        p_savedata_->file << " [smooth_line_points] "
                          << " x "               << smooth_line_[i].x
                          << " y "               << smooth_line_[i].y
                          << " theta "           << smooth_line_[i].theta
                          << " kappa "           << smooth_line_[i].kappa
                          << " s "               << smooth_line_[i].s
                          << " times_smoothing " << times_smoothing_
                          << endl;
    }

    PrintInfo();
}

void QpSplineSmoother::SmootherParaCfg() { // 逻辑 : 判断折线转弯，增量求解2 / 4 m在线拼接，如果rel_s<10.0m,增量2m，如果遇到折线转弯，增量+2m，如果搜到终点，则根据距离调整采点参数
    ++times_smoothing_;

    if (smoother_state_ == SmootherState::init) {
        if (osqp_solver_exitflag_ == 0) {
            smoother_state_ = SmootherState::splicing;
        } else {
            vector<double> pos_robot = {robot_pose_.x, robot_pose_.y};

            int idx = FindNearestPoint(pos_robot, curve_points_);
            idx = (idx >= 1) ? idx - 1 : idx;
            sample_start_point_.x = curve_points_[idx].x;
            sample_start_point_.y = curve_points_[idx].y;
            sample_start_point_.theta = curve_points_[idx].theta;

            len_line_ = len_init_;
        }
    }

    if (smoother_state_ == SmootherState::splicing ||
        smoother_state_ == SmootherState::waiting) {
        if (smooth_line_.back().s - smooth_line_.front().s >= ub_line_len_) {
            smoother_state_ = SmootherState::waiting;
            return;
        } else {
            smoother_state_ = SmootherState::splicing;

            sample_start_point_.x = smooth_line_.back().x;
            sample_start_point_.y = smooth_line_.back().y;
            sample_start_point_.theta = smooth_line_.back().theta;

            len_line_ = len_increment_;
        }
    } else if (smoother_state_ == SmootherState::finished) {
        return;
    }

    double examine_margin = 2.0;
    double len_examine = len_line_ + examine_margin, dis2goal = 0.0;
    vector<double> pos = {sample_start_point_.x, sample_start_point_.y};

    if (GoalPointExamine(pos, len_examine, dis2goal)) {
        len_line_ = dis2goal;
        len_fragment_ = len_line_ / (int(len_line_) + 1);
        interval_sampling_ = len_fragment_ / 5.0;
        interval_sample_ = len_fragment_ / 100.0;

        nums_fragments_ = Double2Int(len_line_ / len_fragment_);
        nums_in_fragment_ = Double2Int(len_fragment_ / interval_sampling_);

        smoother_state_ = SmootherState::finished;
        return;
    }

    if(QuarterTurnExamine(pos, len_examine)) {
        len_line_ = len_examine;
    }
    
    nums_fragments_ = Double2Int(len_line_ / len_fragment_);
    nums_in_fragment_ = Double2Int(len_fragment_ / interval_sampling_);

    if (osqp_solver_exitflag_ != 0 && times_smoothing_ > 1) { // 二次规划求解失败
        if (times_osqp_failed_ > max_times_failed_) {
            return;
        } else {
            max_pos_err_x_ += pos_err_incre_;
            max_pos_err_y_ += pos_err_incre_;
        }
    } else {
        times_osqp_failed_ = 0;
        max_pos_err_x_ = max_pos_err_init_;
        max_pos_err_y_ = max_pos_err_init_;
    }
}

bool QpSplineSmoother::QuarterTurnExamine(
        const vector<double>& point_pos, const double& len_examined) {
    int idx = FindNearestPoint(point_pos, curve_points_);

    double sum_s = 0.0, rel_theta = 0.0;
    vector<double> vec_adjacent_point(2, 0.0);

    while (sum_s <= len_examined && idx < (int)curve_points_.size() - 2) {
        vec_adjacent_point = {
                curve_points_[idx + 1].x - curve_points_[idx].x,
                curve_points_[idx + 1].y - curve_points_[idx].y };
        sum_s += Norm(vec_adjacent_point);

        idx++;

        rel_theta =  curve_points_[idx + 1].theta - curve_points_[idx].theta;
        if (fabs(rel_theta) >= max_turn_angle_) {
            return true;
        }
    }

    return false;
}

int QpSplineSmoother::FindNearestPoint(
        const vector<double>& position, const deque<CurvePoint>& curve_points) {
    int num_points = curve_points.size();

    double dis, dis_pre;
    vector<double> rel_vec(2, 0.0);

    for (int i = 0; i < num_points; ++i) { // 忽略点在s-l系坐标的奇异性
        rel_vec = {
                position[0] - curve_points[i].x,
                position[1] - curve_points[i].y };
        dis = Norm(rel_vec);

        if (i >= 1 && dis >= dis_pre) {
            return i - 1;
        }

        dis_pre = dis;
    }

    return -1;
}

int QpSplineSmoother::FindNearestPoint(
        const vector<double>& position,
        const deque<SmoothLinePoint>& smooth_traj) {
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

bool QpSplineSmoother::GoalPointExamine(
        const vector<double>& point_pos, const double& len_examined,
        double& dis2goal) {
    int idx_goal = curve_points_.size() - 1;

    int idx = FindNearestPoint(point_pos, curve_points_) + 1;

    double sum_s = 0.0;
    vector<double> vec_adjacent_point(2, 0.0);

    while (sum_s <= len_examined) {
        vec_adjacent_point = {
                curve_points_[idx].x - curve_points_[idx - 1].x,
                curve_points_[idx].y - curve_points_[idx - 1].y };
        sum_s += Norm(vec_adjacent_point);

        ++idx;
        
        if (idx == idx_goal) {
            dis2goal = sum_s;
            return true;
        }
    }

    return false;;
}

void QpSplineSmoother::PrintInfo() const {
    return;
    
    for (int i = 0; i < (int)sampling_points_.size(); ++i) {
        cout << "[PrintInfo]" << " sampling_points "
             << sampling_points_[i].x << "  "
             << sampling_points_[i].y << "  "
             << endl;;
    }

    cout << "[PrintInfo] " << " smooth_line_end "
         << smooth_line_.back().x << "  "
         << smooth_line_.back().y << endl;

    for (int i = 0; i < (int)smooth_line_.size(); ++i) {
        cout << "[PrintInfo]" << " smooth_line_points "
             << smooth_line_[i].x << "  " 
             << smooth_line_[i].y << "  "
             << smooth_line_[i].s << "  "
             << endl;;
    }

    cout << "[PrintInfo] " << " para "
         << smoother_state_ - interval_sampling_ << "  "
         << ub_line_len_ << "  "
         << len_line_ << "  "
         << interval_sampling_ << "  "
         << len_fragment_ << endl;
}
