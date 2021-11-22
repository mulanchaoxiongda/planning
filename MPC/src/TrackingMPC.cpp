#include <math.h>
#include <algorithm>
#include <string.h>
#include <fstream>
#include <iostream>
#include <string>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <cassert>
#include <iomanip>
#include <memory>
#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>
#include <iostream>
#include <sys/time.h>

#include "TrackingMPC.h"
#include "CustomFunction.h"

using namespace std;

TrackingMPC::TrackingMPC(RobotModel *p_RobotModel, SaveData *p_savedata):
        TrackingAlgorithm::TrackingAlgorithm(p_RobotModel, p_savedata)
{
    control_start_ = false;

    call_cycle_ = 0.02;

    nx_ = 4;
    nu_ = 2;

    x_.resize(nx_ + nu_);
    matrix_a_.resize(nx_, nx_);
    matrix_b_.resize(nx_, nu_);

    u_.resize(nu_);

    u_max_.resize(nu_, 1);
    u_min_.resize(nu_, 1);
    du_max_.resize(nu_, 1);
    du_min_.resize(nu_, 1);

    ReadInTrajPoints();

    ReadInControlPara();

    GetSensorInfo();

    FindRefPoint(trajectory_points_, sensor_info_);

    running_time_sum_ = 0.0;
    running_time_average_ = 0.0;

    loop_counter_ = 0;

    matrix_A_.resize(nx_ + nu_, nx_ + nu_);
    matrix_B_.resize(nx_ + nu_, nu_);
    matrix_C_.resize(nx_, nx_ + nu_);
}

void TrackingMPC::Reset()
{
    control_start_ = false;
}

ControlCommand TrackingMPC::CalControlCommand(
        vector<TrajPoint> &local_traj_points)
{
    struct timeval t_start, t_end;
    gettimeofday(&t_start,NULL);

    ReadInTrajPoints(local_traj_points);

    GetSensorInfo();

    FindRefPoint(trajectory_points_, sensor_info_);

    if (control_start_ == false) {
        control_start_ = true;

        u_ << sensor_info_.v - reference_point_.v,
              sensor_info_.w - reference_point_.w;
    }

    // CalControlCoefficient(p_RobotModel_->motion_state_.v);
    CalControlCoefficient(); 

    UpdateErrorModel();

    UpdateIncrementModel();

    MatrixXd matrix_phi(nx_ * np_, nx_ + nu_), matrix_theta(nx_ * np_, nu_ * nc_);

    Predict(matrix_phi, matrix_theta);

    MatrixXd matrix_H(nc_ * nu_, nc_ * nu_), matrix_E(nx_ * np_, 1);
    VectorXd matrix_g(nc_ * nu_, 1);

    CalObjectiveFunc(matrix_H, matrix_E, matrix_g, x_, matrix_phi, matrix_theta);

    MatrixXd matrix_A(nc_ * nu_ * 2, nc_ * nu_);
    VectorXd lb(nc_ * nu_ * 2, 1), ub(nc_ * nu_ * 2, 1);

    CalConstraintConditions(matrix_A, lb, ub);

    VectorXd u_optim(nc_ * nu_);

    c_int max_iteration = 200;
    c_float eps_abs = 0.01;

    OptimizationSolver(u_optim, matrix_H, matrix_g, matrix_A, lb, ub,
                       max_iteration, eps_abs);

    u_(0) = u_(0) + u_optim(0);
    u_(1) = u_(1) + u_optim(1);

    cout << u_(0) << " opt " << u_optim(0) << endl;

    ControlCommand control_command = { u_(0) + reference_point_.v,
                                       u_(1) + reference_point_.w,
                                       p_RobotModel_->motion_state_.t };

    cout << control_command.t << "   " << control_command.speed_command << endl;

    p_savedata_->file << "[control_command] "
                      << " Time "             << control_command.t
                      << " speed_command "    << control_command.speed_command
                      << " yawratio_command " << control_command.yaw_rate_command
                      << " t "                << control_command.t << endl;

    gettimeofday(&t_end, NULL);

    loop_counter_++;

    running_time_sum_ = running_time_sum_ + (t_end.tv_sec - t_start.tv_sec) +
                        (double)(t_end.tv_usec - t_start.tv_usec) / 1000000.0;

    running_time_average_ = running_time_sum_ / (double)loop_counter_;

    return control_command;
}

void TrackingMPC::ReadInControlPara()
{
    vector<vector<double>> control_para;

    CustomFunction::txt_to_vectordouble(control_para, "../data/ControlParaMPC.txt");

    v_para_ = control_para[0];

    q_delta_x_para_   = control_para[1];
    q_delta_y_para_   = control_para[2];
    q_delta_yaw_para_ = control_para[3];
    q_delta_wz_para_  = control_para[4];

    r_delta_vc_para_ = control_para[5];
    r_delta_wc_para_ = control_para[6];

    np_para_ = control_para[7];
    nc_para_ = control_para[8];

    control_period_para_ = control_para[9];

    v_min_para_ = control_para[10];
    v_max_para_ = control_para[11];

    w_min_para_ = control_para[12];
    w_max_para_ = control_para[13];

    dv_min_para_ = control_para[14];
    dv_max_para_ = control_para[15];

    dw_min_para_ = control_para[16];
    dw_max_para_ = control_para[17];
}

void TrackingMPC::CalControlCoefficient()
{
    np_ = 15;
    nc_ = 3;

    matrix_q_.resize(nx_, nx_);
    matrix_q_.setIdentity(nx_, nx_);

    matrix_q_(0, 0) = 100.0;
    matrix_q_(1, 1) = 100.0;
    matrix_q_(2, 2) = 1.5;
    matrix_q_(3, 3) = 0.1;

    matrix_r_.resize(nu_, nu_);
    matrix_r_.setIdentity(nu_, nu_);

    matrix_r_(0, 0) = 8.0;
    matrix_r_(1, 1) = 0.2;

    predict_step_ = 0.04;

    u_min_ << -1.5 - reference_point_.v, -40.0 / 57.3 - reference_point_.w;

    u_max_ <<  1.5 - reference_point_.v,  40.0 / 57.3 - reference_point_.w;

    //Todo  du_min_(0): * call_cycle_; du_min_(i)(i >= 1): * predict_step_
    du_min_ << -2.0 * predict_step_,    -120.0 / 57.3 * predict_step_;
    du_max_ <<  2.0 * predict_step_,     120.0 / 57.3 * predict_step_;
}

void TrackingMPC::CalControlCoefficient(double v_sensor)
{
    matrix_q_.setZero(nx_, nx_);

    matrix_q_(0, 0) =
            CustomFunction::interp_linear(v_para_, q_delta_x_para_,   v_sensor);
    matrix_q_(1, 1) =
            CustomFunction::interp_linear(v_para_, q_delta_y_para_,   v_sensor);
    matrix_q_(2, 2) =
            CustomFunction::interp_linear(v_para_, q_delta_yaw_para_, v_sensor);
    matrix_q_(3, 3) =
            CustomFunction::interp_linear(v_para_, q_delta_wz_para_,  v_sensor);

    matrix_r_.setZero(nu_, nu_);

    matrix_r_(0, 0) =
            CustomFunction::interp_linear(v_para_, r_delta_vc_para_, v_sensor);
    matrix_r_(1, 1) =
            CustomFunction::interp_linear(v_para_, r_delta_wc_para_, v_sensor);

    np_ = (int)CustomFunction::interp_linear(v_para_, np_para_, v_sensor);
    nc_ = (int)CustomFunction::interp_linear(v_para_, nc_para_, v_sensor);

    predict_step_ = CustomFunction::interp_linear(v_para_,
                                                   control_period_para_,
                                                   v_sensor);

    double v_min, v_max, w_min, w_max;

    v_min = CustomFunction::interp_linear(v_para_, v_min_para_, v_sensor);
    v_max = CustomFunction::interp_linear(v_para_, v_max_para_, v_sensor);

    w_min = CustomFunction::interp_linear(v_para_, w_min_para_, v_sensor);
    w_max = CustomFunction::interp_linear(v_para_, w_max_para_, v_sensor);

    u_min_ << v_min - reference_point_.v, w_min / 57.3 - reference_point_.w;
    u_max_ << v_max - reference_point_.v, w_max / 57.3 - reference_point_.w;

    double dv_min, dv_max, dw_min, dw_max;

    dv_min = CustomFunction::interp_linear(v_para_, dv_min_para_, v_sensor);
    dv_max = CustomFunction::interp_linear(v_para_, dv_max_para_, v_sensor);

    //Todo  dw_min(0): * call_cycle_; dw_min(i)(i >= 1): * predict_step_
    dw_min = CustomFunction::interp_linear(v_para_, dw_min_para_, v_sensor);
    dw_max = CustomFunction::interp_linear(v_para_, dw_max_para_, v_sensor);

    du_min_ << dv_min * predict_step_, dw_min * predict_step_ / 57.3;
    du_max_ << dv_max * predict_step_, dw_max * predict_step_ / 57.3;
}

void TrackingMPC::UpdateErrorModel()
{
    VectorXd x(nx_), xr(nx_), matrix_kesi(nx_ + nu_);

    x << sensor_info_.x, sensor_info_.y, sensor_info_.yaw, sensor_info_.w;

    xr << reference_point_.x,   reference_point_.y,
          reference_point_.yaw, reference_point_.w;
    
    cout << " x: " << x << endl;
    cout << " xr: " << xr << endl;

    matrix_kesi << x - xr, u_;

    x_ = matrix_kesi;

    cout << x_ << "err" << endl;

    double T1 = 0.07;

    matrix_a_ << 1.0, 0.0, -reference_point_.v * predict_step_ * sin(reference_point_.yaw),  0.0,
                 0.0, 1.0,  reference_point_.v * predict_step_ * cos(reference_point_.yaw),  0.0,
                 0.0, 0.0,  1.0,                                                             predict_step_,
                 0.0, 0.0,  0.0,                                                             1.0 - predict_step_ / T1;

    matrix_b_ << predict_step_ * cos(reference_point_.yaw), 0.0,
                 predict_step_ * sin(reference_point_.yaw), 0.0,
                 0.0,                                       0.0,
                 0.0,                                       predict_step_ / T1;

    p_savedata_->file << " [tracking_error] "
                      << " Time "    << sensor_info_.t
                      << " err_x "   << x_(0)
                      << " err_y "   << x_(1)
                      << " err_yaw " << x_(2)
                      << " t "       << sensor_info_.t << endl;
}

void TrackingMPC::UpdateIncrementModel()
{
    matrix_A_.setZero(nx_ + nu_, nx_ + nu_);

    matrix_A_.block(0,   0,   nx_, nx_) = matrix_a_;
    matrix_A_.block(0,   nx_, nx_, nu_) = matrix_b_;
    matrix_A_.block(nx_, nx_, nu_, nu_) = MatrixXd::Identity(nu_, nu_);

    matrix_B_.setZero(nx_ + nu_, nu_);

    matrix_B_.block(0,   0, nx_, nu_) = matrix_b_;
    matrix_B_.block(nx_, 0, nu_, nu_) = MatrixXd::Identity(nu_, nu_);

    matrix_C_.setZero(nx_, nx_ + nu_);

    matrix_C_.block(0, 0, nx_, nx_) = MatrixXd::Identity(nx_, nx_);
}

void TrackingMPC::Predict(MatrixXd &matrix_phi, MatrixXd &matrix_theta)
{
    matrix_phi.setZero(nx_ * np_, nx_ + nu_);

    for (int i = 0; i < np_; i++) {
        MatrixXd matrix_A_i = MatrixXd::Identity(matrix_A_.rows(), matrix_A_.cols());

        for (int j = 0; j <= i; j++) {
            matrix_A_i = matrix_A_i * matrix_A_;
        }

        matrix_phi.block(i * matrix_C_.rows(), 0,
                         matrix_C_.rows(), matrix_A_i.cols()) =
                matrix_C_ * matrix_A_i;
    }

    matrix_theta.setZero(nx_ * np_, nu_ * nc_);

    for (int i = 0; i < np_; i++) {
        for (int j = 0; j < nc_; j++) {
            if (i >= j) {
                MatrixXd matrix_A_i_substract_j =
                        MatrixXd::Identity(matrix_A_.rows(), matrix_A_.cols());

                for (int k = i - j; k > 0; k--) {
                    matrix_A_i_substract_j = matrix_A_i_substract_j * matrix_A_;
                }

                matrix_theta.block(i * matrix_C_.rows(), j * matrix_B_.cols(),
                                   matrix_C_.rows(), matrix_B_.cols()) =
                        matrix_C_ * matrix_A_i_substract_j * matrix_B_;
            }
        }
    }
}

void TrackingMPC::CalObjectiveFunc(
        MatrixXd &matrix_h, MatrixXd &matrix_e, VectorXd &matrix_g,
        MatrixXd matrix_kesi, MatrixXd matrix_phi, MatrixXd matrix_theta)
{
    MatrixXd matrix_Q(np_ * nx_, np_ * nx_), matrix_R(nc_ * nu_, nc_ * nu_);

    matrix_Q =
            CustomFunction::KroneckerProduct(MatrixXd::Identity(np_, np_),
                                             matrix_q_);

    matrix_R =
            CustomFunction::KroneckerProduct(MatrixXd::Identity(nc_, nc_),
                                             matrix_r_);

    matrix_h = matrix_theta.transpose() * matrix_Q * matrix_theta + matrix_R;
    matrix_h = (matrix_h + matrix_h.transpose()) * 0.5;

    matrix_e = matrix_phi * matrix_kesi;
    matrix_g = ((matrix_e.transpose()) * matrix_Q * matrix_theta).transpose();
}

void TrackingMPC::CalConstraintConditions(
        MatrixXd &matrix_A, VectorXd &lb, VectorXd &ub)
{
    MatrixXd matrix_A_t = MatrixXd::Zero(nc_, nc_);

    for (int i = 0; i < nc_; i++) {
        for (int j = 0; j <= i; j++) {
            matrix_A_t(i, j) = 1;
        }
    }

    MatrixXd A_I(nc_ * nu_, nc_ * nu_);

    A_I = CustomFunction::KroneckerProduct(matrix_A_t, MatrixXd::Identity(nu_, nu_));

    MatrixXd Ut(nc_ * nu_, 1);
    Ut = CustomFunction::KroneckerProduct(MatrixXd::Ones(nc_, 1), u_);

    MatrixXd Umin(nc_ * nu_, 1), Umax(nc_ * nu_, 1);

    Umin = CustomFunction::KroneckerProduct(MatrixXd::Ones(nc_, 1), u_min_);

    Umax = CustomFunction::KroneckerProduct(MatrixXd::Ones(nc_, 1), u_max_);

    MatrixXd delta_Umin(nc_ * nu_, 1), delta_Umax(nc_ * nu_, 1);

    delta_Umin =
            CustomFunction::KroneckerProduct(MatrixXd::Ones(nc_, 1), du_min_);

    delta_Umax =
            CustomFunction::KroneckerProduct(MatrixXd::Ones(nc_, 1), du_max_);

    for (int i = 0; i < nu_; i++) {
        delta_Umin(i, 0) = delta_Umin(i, 0) * call_cycle_ / predict_step_;
        delta_Umax(i, 0) = delta_Umax(i, 0) * call_cycle_ / predict_step_;
    }

    matrix_A.block(0, 0, nc_ * nu_, nc_ * nu_) = A_I;

    matrix_A.block(nc_ * nu_, 0, nc_ * nu_, nc_ * nu_) =
            MatrixXd::Identity(nc_ * nu_, nc_ * nu_);

    lb << Umin - Ut, delta_Umin;

    ub << Umax - Ut, delta_Umax;
}

int TrackingMPC::OptimizationSolver(
        VectorXd &optimal_solution, MatrixXd matrix_p, VectorXd vector_q, 
        MatrixXd matrix_Ac, VectorXd vector_l, VectorXd vector_u,
        c_int max_iteration, c_float eps_abs)
{
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

void TrackingMPC::MatrixToCCS(
        MatrixXd matrix_a, vector<c_float> *sm_x, c_int &sm_nnz,
        vector<c_int> *sm_i, vector<c_int> *sm_p)
{
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

double TrackingMPC::GetRunningTimeAverage()
{
    return running_time_average_;
}
