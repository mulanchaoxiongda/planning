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
    call_cycle_ = 0.02;

    nx_ = 4;
    nu_ = 2;

    x_.resize(nx_ + nu_);
    a_.resize(nx_, nx_);
    b_.resize(nx_, nu_);

    u_.resize(nu_);

    u_max_.resize(nu_, 1);
    u_min_.resize(nu_, 1);
    du_max_.resize(nu_, 1);
    du_min_.resize(nu_, 1);

    ReadInTrajPoints();

    ReadInControlPara();

    GetSensorInfo();

    FindRefPoint(trajectory_points_, sensor_info_);

    u_ << sensor_info_.v-reference_point_.v,
          sensor_info_.w-reference_point_.w;

    running_time_sum_ = 0.0;
    running_time_average_ = 0.0;

    loop_counter_ = 0;

    A_.resize(nx_ + nu_, nx_ + nu_);
    B_.resize(nx_ + nu_, nu_);
    C_.resize(nx_, nx_ + nu_);
}

ControlCommand TrackingMPC::CalControlCommand(vector<TrajPoint> &local_traj_points)
{
    struct timeval t_start, t_end;
    gettimeofday(&t_start,NULL);

    ReadInTrajPoints(local_traj_points);

    GetSensorInfo();

    FindRefPoint(trajectory_points_, sensor_info_);

    CalControlCoefficient(); /* CalControlCoefficient(p_RobotModel_->motion_state_.v); */

    UpdateErrorModel();

    UpdateIncrementModel();

    MatrixXd phi(nx_ * np_, nx_ + nu_), theta(nx_ * np_, nu_ * nc_);

    PredictFunc(phi, theta);

    MatrixXd H(nc_ * nu_, nc_ * nu_), E(nx_ * np_, 1);
    VectorXd g(nc_ * nu_, 1);

    ObjectiveFunc(H, E, g, x_, phi, theta);

    MatrixXd _A(nc_ * nu_ * 2, nc_ * nu_);
    VectorXd lb(nc_ * nu_ * 2, 1), ub(nc_ * nu_ * 2, 1);

    ConstraintCondition(_A, lb, ub);

    int num_constraints = nc_ * nu_ * 2, num_variables = nc_ * nu_;
    VectorXd u_optim(nc_ * nu_);

    c_int max_iteration = 200;
    c_float eps_abs = 0.01;

    OptimizationSolver(u_optim, H, g, _A, lb, ub, num_constraints,
                       num_variables, max_iteration, eps_abs);

    u_(0) = u_(0) + u_optim(0);
    u_(1) = u_(1) + u_optim(1);

    ControlCommand control_command = { u_(0) + reference_point_.v,
                                       u_(1) + reference_point_.w,
                                       p_RobotModel_->motion_state_.t };

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
    np_ = 10;
    nc_ = 5;

    q_.resize(nx_, nx_);
    q_.setIdentity(nx_, nx_);

    q_(0, 0) = 500.0;
    q_(1, 1) = 500.0;
    q_(2, 2) = 1.5;
    q_(3, 3) = 0.1;

    r_.resize(nu_, nu_);
    r_.setIdentity(nu_, nu_);

    r_(0, 0) = 8.0;
    r_(1, 1) = 0.2;

    predict_step_ = 0.04;

    u_min_ << -2.0 - reference_point_.v, -40.0 / 57.3 - reference_point_.w;

    u_max_ <<  2.0 - reference_point_.v,  40.0 / 57.3 - reference_point_.w;

    //Todo  du_min_(0): * call_cycle_; du_min_(i)(i >= 1): * predict_step_
    du_min_ << -1.0 * predict_step_,    -200.0 / 57.3 * predict_step_;
    du_max_ <<  1.0 * predict_step_,     200.0 / 57.3 * predict_step_;
}

void TrackingMPC::CalControlCoefficient(double v_sensor)
{
    q_.setZero(nx_, nx_);

    q_(0, 0) = CustomFunction::interp_linear(v_para_, q_delta_x_para_,   v_sensor);
    q_(1, 1) = CustomFunction::interp_linear(v_para_, q_delta_y_para_,   v_sensor);
    q_(2, 2) = CustomFunction::interp_linear(v_para_, q_delta_yaw_para_, v_sensor);
    q_(3, 3) = CustomFunction::interp_linear(v_para_, q_delta_wz_para_,  v_sensor);

    r_.setZero(nu_, nu_);

    r_(0, 0) = CustomFunction::interp_linear(v_para_, r_delta_vc_para_, v_sensor);
    r_(1, 1) = CustomFunction::interp_linear(v_para_, r_delta_wc_para_, v_sensor);

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
    VectorXd x(nx_), xr(nx_), kesi(nx_ + nu_);

    x << sensor_info_.x, sensor_info_.y, sensor_info_.yaw, sensor_info_.w;

    xr << reference_point_.x,   reference_point_.y,
          reference_point_.yaw, reference_point_.w;

    kesi << x-xr, u_;

    x_ = kesi;

    double T1 = 0.07;

    a_ << 1.0, 0.0, -reference_point_.v*predict_step_*sin(reference_point_.yaw),  0.0,
          0.0, 1.0,  reference_point_.v*predict_step_*cos(reference_point_.yaw),  0.0,
          0.0, 0.0,  1.0,                                                         predict_step_,
          0.0, 0.0,  0.0,                                                         1.0 - predict_step_ / T1;

    b_ << predict_step_ * cos(reference_point_.yaw), 0.0,
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
    A_.setZero(nx_ + nu_, nx_ + nu_);
    A_.block(0, 0, nx_, nx_) = a_;
    A_.block(0, nx_, nx_, nu_) = b_;
    A_.block(nx_, nx_, nu_, nu_) = MatrixXd::Identity(nu_, nu_);

    B_.setZero(nx_ + nu_, nu_);
    B_.block(0, 0, nx_, nu_) = b_;
    B_.block(nx_, 0, nu_, nu_) = MatrixXd::Identity(nu_, nu_);

    C_.setZero(nx_, nx_ + nu_);
    C_.block(0, 0, nx_, nx_) = MatrixXd::Identity(nx_, nx_);
}

void TrackingMPC::PredictFunc(MatrixXd &phi, MatrixXd &theta)
{
    phi.setZero(nx_ * np_, nx_ + nu_);

    for (int i = 0; i < np_; i++) {
        MatrixXd A_i = MatrixXd::Identity(A_.rows(), A_.cols());

        for (int j = 0; j <= i; j++) {
            A_i = A_i * A_;
        }

        phi.block(i * C_.rows(), 0, C_.rows(), A_i.cols()) = C_ * A_i;
    }

    theta.setZero(nx_ * np_, nu_ * nc_);

    for (int i = 0; i < np_; i++) {
        for (int j = 0; j < nc_; j++) {
            if (i >= j) {
                MatrixXd A_i_substract_j =
                        MatrixXd::Identity(A_.rows(), A_.cols());

                for (int k = i - j; k > 0; k--) {
                    A_i_substract_j = A_i_substract_j * A_;
                }

                theta.block(i * C_.rows(), j * B_.cols(), C_.rows(), B_.cols()) =
                        C_ * A_i_substract_j * B_;
            }
        }
    }
}

void TrackingMPC::ObjectiveFunc(
        MatrixXd &h, MatrixXd &e, VectorXd &g,
        MatrixXd kesi, MatrixXd phi, MatrixXd theta)
{
    MatrixXd Q(np_ * nx_, np_ * nx_), R(nc_ * nu_, nc_ * nu_);

    Q = CustomFunction::KroneckerProduct(MatrixXd::Identity(np_, np_), q_);

    R = CustomFunction::KroneckerProduct(MatrixXd::Identity(nc_, nc_), r_);

    h = theta.transpose() * Q * theta + R;
    h = (h+h.transpose()) * 0.5;

    e = phi * kesi;
    g = ((e.transpose()) * Q * theta).transpose();
}

void TrackingMPC::ConstraintCondition(MatrixXd &A, VectorXd &lb, VectorXd &ub)
{
    MatrixXd A_t = MatrixXd::Zero(nc_, nc_);

    for (int i = 0; i < nc_; i++) {
        for (int j = 0; j <= i; j++) {
            A_t(i, j) = 1;
        }
    }

    MatrixXd A_I(nc_ * nu_, nc_ * nu_);

    A_I = CustomFunction::KroneckerProduct(A_t, MatrixXd::Identity(nu_, nu_));

    MatrixXd Ut(nc_ * nu_, 1);
    Ut = CustomFunction::KroneckerProduct(MatrixXd::Ones(nc_, 1), u_);

    MatrixXd Umin(nc_ * nu_, 1), Umax(nc_ * nu_, 1);

    //Todo Umin(0): call_cycle_
    Umin = CustomFunction::KroneckerProduct(MatrixXd::Ones(nc_, 1), u_min_);

    Umax = CustomFunction::KroneckerProduct(MatrixXd::Ones(nc_, 1), u_max_);

    MatrixXd delta_Umin(nc_ * nu_, 1), delta_Umax(nc_ * nu_, 1);

    delta_Umin =
            CustomFunction::KroneckerProduct(MatrixXd::Ones(nc_, 1), du_min_);

    delta_Umax =
            CustomFunction::KroneckerProduct(MatrixXd::Ones(nc_, 1), du_max_);

    for (int i = 0; i < nu_; i++) {
        Umax(i, 0) = Umax(i, 0) * call_cycle_ / predict_step_;
        Umin(i, 0) = Umin(i, 0) * call_cycle_ / predict_step_;

        delta_Umin(i, 0) = delta_Umin(i, 0) * call_cycle_ / predict_step_;
        delta_Umax(i, 0) = delta_Umax(i, 0) * call_cycle_ / predict_step_;
    }

    A.block(0, 0, nc_ * nu_, nc_ * nu_) = A_I;

    A.block(nc_ * nu_, 0, nc_ * nu_, nc_ * nu_) =
            MatrixXd::Identity(nc_ * nu_, nc_ * nu_);

    lb << Umin - Ut, delta_Umin;

    ub << Umax - Ut, delta_Umax;
}

int TrackingMPC::OptimizationSolver(
        VectorXd &optimal_solution, MatrixXd p01, VectorXd q01, MatrixXd Ac,
        VectorXd l01, VectorXd u01, int m01, int n01, c_int max_iteration,
        c_float eps_abs)
{
    vector<c_float> p_x;
    c_int           p_nnz;
    vector<c_int>   p_i;
    vector<c_int>   p_p;

    MatrixToCCS(p01, &p_x, p_nnz, &p_i, &p_p);

    vector<c_float> A_x;
    c_int           A_nnz;
    vector<c_int>   A_i;
    vector<c_int>   A_p;

    MatrixToCCS(Ac, &A_x, A_nnz, &A_i, &A_p);

    int length = q01.size();

    vector<c_float> q(length);

    for (int i = 0; i < length; i++) {
        q.at(i) = q01(i);
    }

    length = l01.size();

    vector<c_float> l(length);

    for (int i = 0; i < length; i++) {
        l.at(i) = l01(i);
    }

    length = u01.size();

    vector<c_float> u(length);

    for (int i = 0; i < length; i++) {
        u.at(i) = u01(i);
    }

    c_int m = Ac.rows();
    c_int n = p01.cols();

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
                           CopyData(A_i), CopyData(A_p));

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
        MatrixXd matrix_, vector<c_float> *sm_x, c_int &sm_nnz,
        vector<c_int> *sm_i, vector<c_int> *sm_p)
{
    sm_p->emplace_back(0);

    int cols = matrix_.cols(), rows = matrix_.rows(), nz = 0;

    if (cols == rows) { // 方阵且对称，取上三角矩阵元素
        for( int j = 0; j < cols; j++) {
            for (int i = 0; i <= j; i++) {
                if(fabs(matrix_(i,j))>0.0000001) {
                    sm_x->emplace_back(matrix_(i,j));
                    sm_i->emplace_back(i);

                    nz++;
                }
            }

            sm_p->emplace_back(nz);
        }
    } else if (cols < rows) { // 非方阵，取全部矩阵元素
        for( int j = 0; j < cols; j++) {
            for (int i = 0; i < rows; i++) {
                if(fabs(matrix_(i,j))>0.0000001) {
                    sm_x->emplace_back(matrix_(i,j));
                    sm_i->emplace_back(i);

                    nz++;
                }
            }

            sm_p->emplace_back(nz);
        }
    }

    sm_nnz = nz;
}
