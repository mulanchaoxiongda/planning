#include <osqp/osqp.h>

#include "qp_spline_smoother.h"

CurveSmoother::CurveSmoother(RobotModel* p_RobotModel, SaveData* p_savedata) {
    p_RobotModel_ = p_RobotModel;
    p_savedata_ = p_savedata;
}

bool CurveSmoother::SetCurvePoints(const vector<CurvePoints>& curve_points) {
    if (curve_points.empty()) {
      return false;
    }

    curve_points_ = curve_points;

    return true;
}

QpSplineSmoother::QpSplineSmoother(RobotModel* p_RobotModel, SaveData* p_savedata) : CurveSmoother(p_RobotModel, p_savedata) {
    vector<CurvePoints> test;
    Txt2Vector(test, "../data/RoutingLine.txt"); // debug
    SetCurvePoints(test);
};

SmootherStatus QpSplineSmoother::GetSmoothCurve(vector<CurvePoints>& smooth_line_points) {
    CalSamplingPoints();

    MatrixXd matrix_h = MatrixXd::Zero(12 * nums_fragments_, 12 * nums_fragments_);
    VectorXd matrix_f = MatrixXd::Zero(12 * nums_fragments_, 1);
    CalObjectiveFunc(matrix_h, matrix_f);
    
    // for (int i = 0; i < matrix_h.rows() / 6; ++i) {
    //     for (int j = 0; j < matrix_h.cols() / 12; ++j) {
    //         cout << i << "  " << j << endl;
    //         cout << matrix_h.block(i * 6, j * 12, 6, 12) << endl << endl;
    //     }
    // }

    // for (int i = 0; i < matrix_h.rows() / 6; ++i) {
    //     cout << i << endl;
    //     cout << matrix_f.block(i * 6, 0, 6, 1) << endl << endl;
    // }

    MatrixXd matrix_a_equ = MatrixXd::Zero(6 + 6 * (nums_fragments_ - 1) + 6, 12 * nums_fragments_);
    MatrixXd matrix_b_equ = MatrixXd::Zero(6 + 6 * (nums_fragments_ - 1) + 6, 1);
    CalEqualityConstraint(matrix_a_equ, matrix_b_equ);
    
    // for (int i = 0; i < matrix_a_equ.rows() / 6; ++i) {
    //     for (int j = 0; j < matrix_a_equ.cols() / 12; ++j) {
    //         cout << i * 6 + 1 << "   " << j * 12 + 1 << " : " << endl;
    //         cout << matrix_a_equ.block(i * 6, j * 12, 6, 12) << endl << endl << endl << endl;
    //     }
    // }
    // cout << matrix_b_equ << endl << endl;

    MatrixXd matrix_a_inequ = MatrixXd::Zero((2 + 2) * (nums_fragments_ * nums_in_fragment_ - 1), 12 * nums_fragments_);
    MatrixXd matrix_b_inequ = MatrixXd::Zero((2 + 2) * (nums_fragments_ * nums_in_fragment_ - 1), 1);
    CalInequalityConstraint(matrix_a_inequ, matrix_b_inequ);
    
    // cout << matrix_a_inequ.rows() << endl;
    // cout << matrix_a_inequ.cols() << endl;
    // for (int i = 0; i < matrix_a_inequ.rows() / 6; ++i) {
    //     for (int j = 0; j < matrix_a_inequ.cols() / 12; ++j) {
    //         cout << i * 6 + 1 << "   " << j * 12 + 1 << " : " << endl;
    //         cout << matrix_a_inequ.block(i * 6, j * 12, 6, 12) << endl << endl << endl << endl;
    //     }
    // }    
    // cout << matrix_b_inequ << endl;
    // cout << matrix_b_inequ.size() << endl;

    VectorXd optimal_coefficient(nums_fragments_ * 12);

    c_int max_iteration = 200;
    c_float eps_abs = 0.001;

    MatrixXd matrix_a = MatrixXd::Zero(matrix_a_equ.rows() + matrix_a_inequ.rows(), matrix_a_equ.cols());
    matrix_a << matrix_a_inequ, matrix_a_equ;
    
    // cout << matrix_a.rows() << "  " << matrix_a.cols() << endl << endl;
    // for (int i = 0; i < matrix_a.rows() / 6; ++i) {
    //     for (int j = 0; j < matrix_a.cols() / 12; ++j) {
    //         cout << i * 6 + 1 << "  " << j * 12 + 1 <<endl;
    //         cout << matrix_a.block(i * 6, j * 12, 6, 12) << endl << endl;
    //     }
    // }

    VectorXd vector_lb = VectorXd::Zero(matrix_b_equ.rows() + matrix_b_inequ.rows());
    VectorXd matrix_bl_inequ(matrix_b_inequ.rows());
    for (int i = 0; i < matrix_b_inequ.rows(); ++i) {
        matrix_bl_inequ(i) = -10000000000.0;
    }
    vector_lb << matrix_bl_inequ, matrix_b_equ; // to do : optimize interface
    
    // for (int i = 0; i < vector_lb.size() / 6; ++i) {
    //     cout << i * 6 + 1 << "  " << 1 << endl;
    //     cout << vector_lb.block(i * 6, 0, 6, 1) << endl << endl << endl;
    // }
    
    VectorXd vector_ub = VectorXd::Zero(matrix_b_equ.rows() + matrix_b_inequ.rows());
    VectorXd matrix_bu_inequ(matrix_b_inequ.rows());
    matrix_bu_inequ = matrix_b_inequ;
    vector_ub << matrix_bu_inequ, matrix_b_equ;
    
    // for (int i = 0; i < vector_lb.size() / 6; ++i) {
    //     cout << i * 6 + 1 << "  " << 1 << endl;
    //     cout << vector_ub.block(i * 6, 0, 6, 1) << endl << endl << endl;
    // }

    OptimizationSolver(optimal_coefficient, matrix_h, matrix_f, matrix_a, vector_lb, vector_ub, max_iteration, eps_abs);
    
    // cout << optimal_coefficient.rows() << "   " << optimal_coefficient.cols() << endl << endl;
    // cout << optimal_coefficient << endl;

    CalSmoothTraj(optimal_coefficient);

    return SmootherStatus::success;
}

void QpSplineSmoother::CalSamplingPoints() {
    int num_points = curve_points_.size();

    double s = 0.0;
    accumulative_length_.push_back(s);
    pos_x_.push_back(curve_points_.at(0).x);
    pos_y_.push_back(curve_points_.at(0).y);
    pos_theta_.push_back(curve_points_.at(0).theta);

    for (int i = 1; i < num_points; ++i) {
        double rel_dis = sqrt(pow(curve_points_.at(i).x - curve_points_.at(i - 1).x, 2.0) + pow(curve_points_.at(i).y - curve_points_.at(i - 1).y, 2.0));
        s += rel_dis;

        accumulative_length_.push_back(s);
        pos_x_.push_back(curve_points_.at(i).x);
        pos_y_.push_back(curve_points_.at(i).y);
        pos_theta_.push_back(curve_points_.at(i).theta);
    }

    double len = 0.0, len_max = floor(s), min_value = 0.001;
    while (len <= len_line_ + min_value && len <= len_max + min_value) {
        double x = InterpLinear(accumulative_length_, pos_x_, len);
        double y = InterpLinear(accumulative_length_, pos_y_, len);
        double theta = InterpLinear(accumulative_length_, pos_theta_, len);

        sampling_points_.push_back({x, y, theta, len});

        len += interval_sampling_;
    }
}

void QpSplineSmoother::CalObjectiveFunc(MatrixXd& matrix_h, VectorXd& matrix_f) {
    double s = len_fragment_;

    for (int i = 0; i < nums_fragments_; ++i) {
        MatrixXd matrix_h_acc = MatrixXd::Zero(6, 6);
        matrix_h_acc.block(2, 2, 4, 4) <<  4.0 * pow(s, 1.0),    6.0 * pow(s, 2.0),            8.0 * pow(s, 3.0),           10.0 * pow(s, 4.0),
                                           6.0 * pow(s, 2.0),   12.0 * pow(s, 3.0),           18.0 * pow(s, 4.0),           24.0 * pow(s, 5.0),
                                           8.0 * pow(s, 3.0),   18.0 * pow(s, 4.0),  (144.0 / 5.0) * pow(s, 5.0),           40.0 * pow(s, 6.0),
                                          10.0 * pow(s, 4.0),   24.0 * pow(s, 5.0),           40.0 * pow(s, 6.0),  (400.0 / 7.0) * pow(s, 7.0);

        MatrixXd matrix_h_jerk = MatrixXd::Zero(6, 6);
        matrix_h_jerk.block(3, 3, 3, 3) <<  36.0 * pow(s, 1.0),    72.0 * pow(s, 2.0),   120.0 * pow(s, 3.0),
                                            72.0 * pow(s, 2.0),   192.0 * pow(s, 3.0),   360.0 * pow(s, 4.0),
                                           120.0 * pow(s, 3.0),   360.0 * pow(s, 4.0),   720.0 * pow(s, 5.0);

        MatrixXd matrix_h_pos = MatrixXd::Zero(6, 6);
        matrix_h_pos << pow(s, 0.0),  pow(s, 1.0),  pow(s, 2.0),  pow(s, 3.0),  pow(s, 4.0),  pow(s, 5.0),
                        pow(s, 1.0),  pow(s, 2.0),  pow(s, 3.0),  pow(s, 4.0),  pow(s, 5.0),  pow(s, 6.0),
                        pow(s, 2.0),  pow(s, 3.0),  pow(s, 4.0),  pow(s, 5.0),  pow(s, 6.0),  pow(s, 7.0),
                        pow(s, 3.0),  pow(s, 4.0),  pow(s, 5.0),  pow(s, 6.0),  pow(s, 7.0),  pow(s, 8.0),
                        pow(s, 4.0),  pow(s, 5.0),  pow(s, 6.0),  pow(s, 7.0),  pow(s, 8.0),  pow(s, 9.0),
                        pow(s, 5.0),  pow(s, 6.0),  pow(s, 7.0),  pow(s, 8.0),  pow(s, 9.0),  pow(s, 10.0);

        MatrixXd matrix_hx = MatrixXd::Zero(6, 6), matrix_hy = MatrixXd::Zero(6, 6);
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

        MatrixXd matrix_fx = MatrixXd::Zero(6, 1), matrix_fy = MatrixXd::Zero(6, 1), matrix_f_xy = MatrixXd::Zero(6, 1);
        matrix_f_xy << 1, pow(s, 1.0), pow(s, 2.0), pow(s, 3.0), pow(s, 4.0), pow(s, 5.0);
        matrix_fx = -2.0 * sampling_points_.at((i + 1) * 5).x * weight_pos_x_ * matrix_f_xy;
        matrix_fy = -2.0 * sampling_points_.at((i + 1) * 5).y * weight_pos_y_ * matrix_f_xy;

        matrix_f.block(idx, 0, 6, 1) =  matrix_fx;
        matrix_f.block(idx + 6, 0, 6, 1) =  matrix_fy;
    }

    matrix_h = (matrix_h + matrix_h.transpose()) * 0.5; // debug
}

void QpSplineSmoother::CalEqualityConstraint(MatrixXd& matrix_a_equ, MatrixXd& matrix_b_equ) {
    double s = 0.0;

    MatrixXd matrix_ax_start = MatrixXd::Zero(3, 12);
    matrix_ax_start.block(0, 0, 3, 6) << 1.0,    s,     pow(s, 2.0),  pow(s, 3.0),        pow(s, 4.0),         pow(s, 5.0),
                                         0.0,    1.0,   2.0 * s,      3.0 * pow(s, 2.0),  4.0 * pow(s, 3.0),   5.0 * pow(s, 4.0),
                                         0.0,    0.0,   2.0,          6.0 * s,            12.0 * pow(s, 2.0),  20.0 * pow(s, 3.0);

    MatrixXd matrix_ay_start = MatrixXd::Zero(3, 12);
    matrix_ay_start.block(0, 6, 3, 6) << 1.0,    s,     pow(s, 2.0),  pow(s, 3.0),        pow(s, 4.0),         pow(s, 5.0),
                                         0.0,    1.0,   2.0 * s,      3.0 * pow(s, 2.0),  4.0 * pow(s, 3.0),   5.0 * pow(s, 4.0),
                                         0.0,    0.0,   2.0,          6.0 * s,            12.0 * pow(s, 2.0),  20.0 * pow(s, 3.0);

    matrix_a_equ.block(0, 0, 3, 12) << matrix_ax_start;
    matrix_a_equ.block(3, 0, 3, 12) << matrix_ay_start;

    double x_start, y_start, vx_start, vy_start, ax_start, ay_start;
    x_start = sampling_points_.at(0).x;
    y_start = sampling_points_.at(0).y;
    vx_start = cos(sampling_points_.at(0).theta);
    vy_start = sin(sampling_points_.at(0).theta);
    ax_start = 0.0;
    ay_start = 0.0;
    
    matrix_b_equ.block(0, 0, 6, 1) << x_start, vx_start, ax_start, y_start, vy_start, ay_start;

    s = len_fragment_;

    MatrixXd matrix_ax_end = MatrixXd::Zero(3, 12);
    matrix_ax_end.block(0, 0, 3, 6) << 1.0,    s,     pow(s, 2.0),  pow(s, 3.0),        pow(s, 4.0),         pow(s, 5.0),
                                       0.0,    1.0,   2.0 * s,      3.0 * pow(s, 2.0),  4.0 * pow(s, 3.0),   5.0 * pow(s, 4.0),
                                       0.0,    0.0,   2.0,          6.0 * s,            12.0 * pow(s, 2.0),  20.0 * pow(s, 3.0);

    MatrixXd matrix_ay_end = MatrixXd::Zero(3, 12);
    matrix_ay_end.block(0, 6, 3, 6) << 1.0,    s,     pow(s, 2.0),  pow(s, 3.0),        pow(s, 4.0),         pow(s, 5.0),
                                       0.0,    1.0,   2.0 * s,      3.0 * pow(s, 2.0),  4.0 * pow(s, 3.0),   5.0 * pow(s, 4.0),
                                       0.0,    0.0,   2.0,          6.0 * s,            12.0 * pow(s, 2.0),  20.0 * pow(s, 3.0);

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

    matrix_b_equ.block(idx_row_end - 5, 0, 6, 1) << x_end, vx_end, ax_end, y_end, vy_end, ay_end;

    MatrixXd matrix_a_x_continuity = MatrixXd::Zero(3, 24), matrix_a_y_continuity = MatrixXd::Zero(3, 24);

    for (int i = 0; i < nums_fragments_ - 1; ++i) {
        s = len_fragment_;
        
        matrix_a_x_continuity.block(0, 0, 3, 6) << 1.0,    s,     pow(s, 2.0),  pow(s, 3.0),        pow(s, 4.0),         pow(s, 5.0),
                                                  0.0,    1.0,   2.0 * s,      3.0 * pow(s, 2.0),  4.0 * pow(s, 3.0),   5.0 * pow(s, 4.0),
                                                  0.0,    0.0,   2.0,          6.0 * s,            12.0 * pow(s, 2.0),  20.0 * pow(s, 3.0);

        matrix_a_y_continuity.block(0, 6, 3, 6) << 1.0,    s,     pow(s, 2.0),  pow(s, 3.0),        pow(s, 4.0),         pow(s, 5.0),
                                                   0.0,    1.0,   2.0 * s,      3.0 * pow(s, 2.0),  4.0 * pow(s, 3.0),   5.0 * pow(s, 4.0),
                                                   0.0,    0.0,   2.0,          6.0 * s,            12.0 * pow(s, 2.0),  20.0 * pow(s, 3.0);
        
        s = 0.0;
        
        matrix_a_x_continuity.block(0, 12, 3, 6) << 1.0,    s,     pow(s, 2.0),  pow(s, 3.0),        pow(s, 4.0),         pow(s, 5.0),
                                                    0.0,    1.0,   2.0 * s,      3.0 * pow(s, 2.0),  4.0 * pow(s, 3.0),   5.0 * pow(s, 4.0),
                                                    0.0,    0.0,   2.0,          6.0 * s,            12.0 * pow(s, 2.0),  20.0 * pow(s, 3.0);
        matrix_a_x_continuity.block(0, 12, 3, 6) = -1.0 * matrix_a_x_continuity.block(0, 12, 3, 6);

        matrix_a_y_continuity.block(0, 18, 3, 6) << 1.0,    s,     pow(s, 2.0),  pow(s, 3.0),        pow(s, 4.0),         pow(s, 5.0),
                                                    0.0,    1.0,   2.0 * s,      3.0 * pow(s, 2.0),  4.0 * pow(s, 3.0),   5.0 * pow(s, 4.0),
                                                    0.0,    0.0,   2.0,          6.0 * s,            12.0 * pow(s, 2.0),  20.0 * pow(s, 3.0);
        matrix_a_y_continuity.block(0, 18, 3, 6) = -1.0 * matrix_a_y_continuity.block(0, 18, 3, 6);

        int idx_row = 6 * (i + 1), idx_col = 12 * i;
        matrix_a_equ.block(idx_row, idx_col, 3, 24) = matrix_a_x_continuity;
        matrix_a_equ.block(idx_row + 3, idx_col, 3, 24) = matrix_a_y_continuity;
    }
}

void QpSplineSmoother::CalInequalityConstraint(MatrixXd& matrix_a_inequ, MatrixXd& matrix_b_inequ) {
    MatrixXd matrix_ax_inequ = MatrixXd::Zero(2, 12), matrix_ay_inequ = MatrixXd::Zero(2, 12);

    double s = 0;
    
    for (int i = 0; i < nums_fragments_; ++i) {
        for (int j = 0; j < nums_in_fragment_; ++j) {
            if (i == nums_fragments_ - 1 && j == nums_in_fragment_ - 1) {
                break;
            }

            matrix_ax_inequ.setZero(2, 12);
            matrix_ay_inequ.setZero(2, 12);

            s = double(j + 1) / nums_in_fragment_ * len_fragment_;

            matrix_ax_inequ.block(0, 0, 1, 6) << 1.0,  s,  pow(s, 2.0), pow(s, 3.0), pow(s, 4.0), pow(s, 5.0);
            matrix_ax_inequ.block(1, 0, 1, 6) = -1.0 * matrix_ax_inequ.block(0, 0, 1, 6);

            matrix_ay_inequ.block(0, 6, 1, 6) << 1.0,  s,  pow(s, 2.0), pow(s, 3.0), pow(s, 4.0), pow(s, 5.0);
            matrix_ay_inequ.block(1, 6, 1, 6) = -1.0 * matrix_ax_inequ.block(0, 0, 1, 6);

            int idx_row, idx_col;
            idx_row = nums_in_fragment_ * (2 + 2) * i + (2 + 2) * j;
            idx_col = 12 * i;

            matrix_a_inequ.block(idx_row, idx_col, 2, 12) = matrix_ax_inequ;
            matrix_a_inequ.block(idx_row + 2, idx_col, 2, 12) = matrix_ay_inequ;

            int idx_ref_point = nums_in_fragment_ * i + j + 1;
            double x_ref = sampling_points_.at(idx_ref_point).x;
            double y_ref = sampling_points_.at(idx_ref_point).y;

            MatrixXd matrix_bx_inequ = MatrixXd::Zero(2, 1), matrix_by_inequ = MatrixXd::Zero(2, 1);
            matrix_bx_inequ << x_ref + max_pos_err_x_, -x_ref + max_pos_err_x_;
            matrix_by_inequ << y_ref + max_pos_err_y_, -y_ref + max_pos_err_y_;

            matrix_b_inequ.block(idx_row, 0, 2, 1) = matrix_bx_inequ;
            matrix_b_inequ.block(idx_row + 2, 0, 2, 1) = matrix_by_inequ;
        }
    }
}

int QpSplineSmoother::OptimizationSolver(
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

    c_int m = matrix_Ac.rows(); // num of constraints
    c_int n = matrix_p.cols(); // num of variables

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

double QpSplineSmoother::InterpLinear(vector<double>& x, vector<double>& y, double x0)
{
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

void QpSplineSmoother::Txt2Vector(vector<CurvePoints>& res, string pathname)
{
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

            res.push_back({temp[0], temp[1], temp[2]}); // {x, y, theta}
            temp.clear();
            string_.clear();
        }
    }

    read_file.close();
}

void QpSplineSmoother::CalSmoothTraj(VectorXd& poly_coefficient) {
    vector<double> poly_coef(12, 0);
    double s = 0.0, x, y;

    smooth_line_.clear(); // debug

    for (int i = 0; i < nums_fragments_; ++i) {
        for (int j = 0; j < 12; ++j) {
            poly_coef[j] = poly_coefficient[i * 12 + j];
        }

        if (i != 0) {
            s = interval_sampling_;
        }

        while (s < len_fragment_ + 0.001) {
            x = poly_coef[0] + poly_coef[1] * s + poly_coef[2] * pow(s, 2.0) + poly_coef[3] * pow(s, 3.0) + poly_coef[4] * pow(s, 4.0) + poly_coef[5] * pow(s, 5.0);
            y = poly_coef[6] + poly_coef[7] * s + poly_coef[8] * pow(s, 2.0) + poly_coef[9] * pow(s, 3.0) + poly_coef[10] * pow(s, 4.0) + poly_coef[11] * pow(s, 5.0);

            smooth_line_.push_back({x, y, s}); // debug

            s += interval_sampling_;
        }
    }
    
    cout << endl << endl;
    for (int i = 0; i < smooth_line_.size(); ++i) {
        cout << smooth_line_[i].x << "  " << smooth_line_[i].y << "  " << smooth_line_[i].theta << endl;
    }
}
