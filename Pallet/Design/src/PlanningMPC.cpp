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

#include "PlanningMPC.h"
#include "CustomFunction.h"

using namespace std;

PlanningMPC::PlanningMPC(RobotModel *p_robot_model, SaveData *p_savedata):
        PlanningAlgorithm::PlanningAlgorithm(p_robot_model, p_savedata)
{
    start_gate_ = false;

    weak_planning_num_ = 3;

    call_cycle_ = 0.1;

    nx_ = 4;
    nu_ = 2;

    matrix_kesi_.resize(nx_ + nu_);
    matrix_a_.resize(nx_, nx_);
    matrix_b_.resize(nx_, nu_);

    u_max_.resize(nu_, 1);
    u_min_.resize(nu_, 1);
    du_max_.resize(nu_, 1);
    du_min_.resize(nu_, 1);

    u_pre_.resize(nu_);

    running_time_sum_ = 0.0;

    loop_counter_ = 0;

    gate_start_ = true;

    matrix_A_.resize(nx_ + nu_, nx_ + nu_);
    matrix_B_.resize(nx_ + nu_, nu_);
    matrix_C_.resize(nx_, nx_ + nu_);

    ref_point_command_.resize(nc_ * nu_);
}

ControlCommand PlanningMPC::CalRefTrajectory(
        vector<TrajPoint> &local_traj_points)
{
    ControlCommand result = {0.0, 0.0, 0.0};

    return result;
}

ControlCommand PlanningMPC::CalRefTrajectory(
        vector<TrajPoint> &optimal_traj_points,
        vector<TrajPoint> &local_traj_points)
{
    struct timeval t_start, t_end;
    gettimeofday(&t_start,NULL);

    local_traj_points_.assign(
            local_traj_points.begin(), local_traj_points.end());

    if (start_gate_ == false) {
        GetSensorInfo();

        /* 控制量初始值求解方案A */
        u_pre_ << sensor_info_.v - local_ref_traj_point_.v,
                  sensor_info_.w - local_ref_traj_point_.w;

        memcpy(&sensor_info_planner_, &sensor_info_, sizeof(SensorInfo));
    } else {
        UpdatePlannerSensorInfo();
    
        /* 控制量初始值求解方案B：在规划轨迹的连续性上，方案B优于方案A，源于强、弱规划的
        衔接策略 + 运动学线性化误差模型，方案B规划的轨迹更顺化，易于跟踪，小车对全局路径的
        跟踪精度也会更高 */
        u_pre_ << u_opt_storage_(index_init_point_strong_planner_ * nu_),
                  u_opt_storage_(index_init_point_strong_planner_ * nu_ + 1);
    }

    FindRefPoint();

    VectorXd u_min(nu_, 1), u_max(nu_, 1), du_min(nu_, 1), du_max(nu_, 1);

    /* CalControlCoefficient(sensor_info_planner_.v); */
    CalControlCoefficient();

    UpdateErrorModel();

    UpdateIncrementModel();

    MatrixXd matrix_phi(nx_ * np_, nx_ + nu_);
    MatrixXd matrix_theta(nx_ * np_, nu_ * nc_);

    Predict(matrix_phi, matrix_theta);

    MatrixXd matrix_H(nc_ * nu_, nc_ * nu_), matrix_E(nx_ * np_, 1);
    VectorXd matrix_g(nc_ * nu_, 1);

    CalObjectiveFunc(
            matrix_H, matrix_E, matrix_g, matrix_phi, matrix_theta);

    MatrixXd matrix_A(nc_ * nu_ * 2, nc_ * nu_);
    VectorXd lb(nc_ * nu_ * 2, 1), ub(nc_ * nu_ * 2, 1);

    CalConstraintConditions(matrix_A, lb, ub);

    VectorXd u_opt(nc_ * nu_);

    c_int max_iteration = 200;
    c_float eps_abs = 0.01;

    OptimizationSolver(u_opt, matrix_H, matrix_g, matrix_A,
                       lb, ub, max_iteration, eps_abs);

    u_optimal_.resize(nc_ * nu_, 1);
    u_opt_storage_.resize(nc_ * nu_, 1);

    MatrixXd matrix_A_t = MatrixXd::Zero(nc_, nc_);

    for (int i = 0; i < nc_; i++) {
        for (int j = 0; j <= i; j++) {
            matrix_A_t(i, j) = 1;
        }
    }

    MatrixXd matrix_matrix_A_i(nc_ * nu_, nc_ * nu_);
    matrix_matrix_A_i =
            CustomFunction::KroneckerProduct(matrix_A_t,
                                             MatrixXd::Identity(nu_, nu_));

    MatrixXd Ut(nc_ * nu_, 1);
    Ut = CustomFunction::KroneckerProduct(MatrixXd::Ones(nc_, 1), u_pre_);

    u_optimal_ = Ut + matrix_matrix_A_i * u_opt;

    u_opt_storage_ = u_optimal_;

    u_pre_(0) = u_optimal_(0);
    u_pre_(1) = u_optimal_(1);

    VectorXd ref_point_command;
    ref_point_command.resize(nc_ * nu_);

    /* 线性化误差模型+osqp库的缺陷在这里，参考轨迹的速度、
    角速度变化时，预测轨迹误差较大，但不影响避障可行性 */
    CalPredictForwardCommand();

    for (int i = 0; i < nc_; i++) {
        for (int j = 0; j < nu_; j++) {
            u_optimal_(nu_ * i + j) =
                    u_optimal_(nu_ * i + j) +
                    ref_point_command_.at(nu_ * i + j);    
        }
    }
    
    UpdateReferenceTrajectory();

    optimal_traj_points.assign(opt_traj_points_.begin(),
                               opt_traj_points_.end());

    gettimeofday(&t_end, NULL);

    loop_counter_++;

    running_time_sum_ = running_time_sum_ + (t_end.tv_sec - t_start.tv_sec) +
                        (double)(t_end.tv_usec - t_start.tv_usec) / 1000000.0;


    running_time_average_ = running_time_sum_ / (double)loop_counter_;

    p_savedata_->file << "[planning_control_command] "
                          << " Time " << sensor_info_.t
                          << " vc "   << u_optimal_(0)
                          << " wc "   << u_optimal_(1)
                          << " Time " << sensor_info_planner_.t << endl;

    ControlCommand result = {u_optimal_(0), u_optimal_(1), sensor_info_.t};

    return result;
}

void PlanningMPC::ReadInGoalTraj()
{
    ;
}

void PlanningMPC::CollisionDetection()
{
    ;
}

void PlanningMPC::FindRefPoint()
{
    int size_ref_traj = local_traj_points_.size();

    if (size_ref_traj <= 1 ) {
        cout << "[error] local reference trajectory has only "
             << size_ref_traj << " points " << endl << endl;
    }


    vector<double> relative_time;
    vector<double> fabs_relative_time;

    for (int i = 0; i < size_ref_traj; i++) {
        double delta_t, fabs_delta_t;

        delta_t = sensor_info_planner_.t - local_traj_points_.at(i).t_ref;
        fabs_delta_t = -1.0 * fabs(delta_t);

        relative_time.push_back(delta_t);
        fabs_relative_time.push_back(fabs_delta_t);
    }

    vector<double>::iterator biggest =
            max_element(begin(fabs_relative_time), end(fabs_relative_time));

    int ref_point_index;

    ref_point_index = distance(begin(fabs_relative_time), biggest);

    if (relative_time.at(ref_point_index) <= 0.0) {
        ref_point_index = ref_point_index - 1;

        if(ref_point_index == -1) {
            ref_point_index = 0;
        }
    } else {
        if (ref_point_index == (size_ref_traj - 1)) {
            ref_point_index = size_ref_traj - 2;
        }
    }

    double dis1, dis2, dis_total;

    dis1 = fabs_relative_time.at(ref_point_index);
    dis2 = fabs_relative_time.at(ref_point_index + 1);

    dis_total = dis1 + dis2;

    local_ref_traj_point_.t =
            ( local_traj_points_.at(ref_point_index).t_ref * dis2 +
              local_traj_points_.at(ref_point_index + 1).t_ref * dis1 ) /
              dis_total;
    
    // Todo: 在v的求解中，暂未引入a的影响
    local_ref_traj_point_.v =
            ( local_traj_points_.at(ref_point_index).v_ref * dis2 +
              local_traj_points_.at(ref_point_index + 1).v_ref * dis1 ) /
            dis_total;
    
    local_ref_traj_point_.w =
            ( local_traj_points_.at(ref_point_index).w_ref * dis2 +
              local_traj_points_.at(ref_point_index + 1).w_ref * dis1 ) /
              dis_total;

    // 调试用，验证robot_model中AGV加速度解算的正确性，规划与控制中暂未使用加速度变量
    local_ref_traj_point_.ax =
            ( local_traj_points_.at(ref_point_index).ax_ref * dis2 +
              local_traj_points_.at(ref_point_index + 1).ax_ref * dis1 ) /
              dis_total;

    local_ref_traj_point_.ay =
            ( local_traj_points_.at(ref_point_index).ay_ref * dis2 +
              local_traj_points_.at(ref_point_index + 1).ay_ref * dis1 ) /
              dis_total;
    
    double delta_t1, delta_t2;
    
    delta_t1 = 
            local_ref_traj_point_.t -
            local_traj_points_.at(ref_point_index).t_ref;

    delta_t2 = 
            local_traj_points_.at(ref_point_index + 1).t_ref -
            local_ref_traj_point_.t;
    
    local_ref_traj_point_.yaw =
            ((local_traj_points_.at(ref_point_index).yaw_ref +
            local_traj_points_.at(ref_point_index).w_ref * delta_t1) *
            dis2 + (local_traj_points_.at(ref_point_index + 1).yaw_ref -
            local_traj_points_.at(ref_point_index + 1).w_ref * delta_t2) *
            dis1 ) / dis_total;
    
    local_ref_traj_point_.x =
            ((local_traj_points_.at(ref_point_index).x_ref +
            local_traj_points_.at(ref_point_index).v_ref * delta_t1 *
            cos(local_traj_points_.at(ref_point_index).yaw_ref * 0.5 +
            local_ref_traj_point_.yaw * 0.5)) * dis2 +
            (local_traj_points_.at(ref_point_index + 1).x_ref -
            local_traj_points_.at(ref_point_index + 1).v_ref * delta_t2 *
            cos(local_traj_points_.at(ref_point_index + 1).yaw_ref * 0.5 +
            local_ref_traj_point_.yaw * 0.5)) * dis1 ) / dis_total;
    
    local_ref_traj_point_.y =
            ((local_traj_points_.at(ref_point_index).y_ref +
            local_traj_points_.at(ref_point_index).v_ref * delta_t1 *
            sin(local_traj_points_.at(ref_point_index).yaw_ref * 0.5 +
            local_ref_traj_point_.yaw * 0.5)) * dis2 +
            (local_traj_points_.at(ref_point_index + 1).y_ref -
            local_traj_points_.at(ref_point_index + 1).v_ref * delta_t2 *
            sin(local_traj_points_.at(ref_point_index + 1).yaw_ref * 0.5 +
            local_ref_traj_point_.yaw * 0.5)) * dis1 ) / dis_total;

    p_savedata_->file << "[plainning_global_reference_point] "
                      << " Time "    << sensor_info_.t
                      << " x_ref "   << local_ref_traj_point_.x
                      << " y_ref "   << local_ref_traj_point_.y
                      << " yaw_ref " << local_ref_traj_point_.yaw
                      << " v_ref "   << local_ref_traj_point_.v
                      << " w_ref "   << local_ref_traj_point_.w
                      << " t "       << local_ref_traj_point_.t
                      << endl;
}

void PlanningMPC::UpdatePlannerSensorInfo()
{
    int size_ref_traj = opt_traj_points_.size();

    if (size_ref_traj <= 1 ) {
        cout << "[error] local reference trajectory has only "
             << size_ref_traj << " points " << endl << endl;
    }

    vector<double> relative_time;
    vector<double> fabs_relative_time;

    for (int i = 0; i < size_ref_traj; i++) {
        double delta_t, fabs_delta_t;

        delta_t =
                p_robot_model_->motion_state_.t - opt_traj_points_.at(i).t_ref;

        fabs_delta_t = -1.0 * fabs(delta_t);

        relative_time.push_back(delta_t);
        fabs_relative_time.push_back(fabs_delta_t);
    }

    vector<double>::iterator biggest =
            max_element(begin(fabs_relative_time), end(fabs_relative_time));

    int ref_point_index;

    ref_point_index = distance(begin(fabs_relative_time), biggest);

    if (relative_time.at(ref_point_index) <= 0.0) {
        ref_point_index = ref_point_index - 1;

        if(ref_point_index == -1) {
            ref_point_index = 0;
        }
    } else {
        ref_point_index = ref_point_index;

        if (ref_point_index == (size_ref_traj - 1)) {
            ref_point_index = size_ref_traj - 2;
        }
    }

    int index_transition_point = ref_point_index + weak_planning_num_;
    
    sensor_info_planner_.t   =
            opt_traj_points_.at(index_transition_point).t_ref;
    sensor_info_planner_.x   =
            opt_traj_points_.at(index_transition_point).x_ref;
    sensor_info_planner_.y   =
            opt_traj_points_.at(index_transition_point).y_ref;
    sensor_info_planner_.yaw =
            opt_traj_points_.at(index_transition_point).yaw_ref;
    sensor_info_planner_.v   =
            opt_traj_points_.at(index_transition_point).v_ref;
    sensor_info_planner_.w   =
            opt_traj_points_.at(index_transition_point).w_ref;

    planner_sensor_info_id_ = ref_point_index;
    index_init_point_strong_planner_ = index_transition_point - 1;
}

void PlanningMPC::CalControlCoefficient()
{

    np_ = 15;
    nc_ = 6;

    matrix_q_.resize(nx_, nx_);
    matrix_q_.setIdentity(nx_, nx_);
 
    double k_lon, k_lat;

    // 规划层位置误差权重较控制层的小，起到平滑轨迹的作用
    // 横向运动依存与纵向运动，与纵平面、水平面制导关系相似
    // 不宜过小，会导致规划轨迹不经过目标点
    k_lon = 40.0; // 纵向运动位置误差权重
    k_lat = 40.0; // 横向运动位置误差权重

    double psi;

    psi = sensor_info_.yaw;

    double psi_trans;

    if (fabs(psi) >= M_PI / 2.0) {
        psi_trans = M_PI - fabs(psi);
    } else {
        psi_trans = fabs(psi);
    }
    
    double k_x, k_y;

    // 分解公式有待进一步验证，目前推演暂未发现问题
    k_x = k_lon * cos(psi_trans) + k_lat * sin(psi_trans);
    k_y = k_lon * sin(psi_trans) + k_lat * cos(psi_trans);

    matrix_q_(0, 0) = k_x;
    matrix_q_(1, 1) = k_y;
    matrix_q_(2, 2) = 1.5;
    matrix_q_(3, 3) = 2.0;

    matrix_r_.resize(nu_, nu_);
    matrix_r_.setIdentity(nu_, nu_);

    matrix_r_(0, 0) = 8.0;
    matrix_r_(1, 1) = 0.2;

    predict_step_ = 0.08;

    /* Todo： 无人车控制的约束项应扣除参考轨迹的速度、角速度、加速度、角加速度
    Todo： 规划中的约束项应扣除各预测点的参考速度、角速度、加速度、角加速度
    Todo： 约束项扣除的参开速度、角速度应随预测点调整，预测模型A应随预测点调整
    说明： 如果是非线性模型、非线性优化，则不需要以上处理 */
    u_min_ << -1.0 - local_ref_traj_point_.v,
              -35.0 / 57.3 - local_ref_traj_point_.w;

    u_max_ <<  1.0 - local_ref_traj_point_.v,
               35.0 / 57.3 - local_ref_traj_point_.w;

    /* du_min_(0): * call_cycle_; du_min_(i)(i >= 1): * predict_step_ */
    du_min_ << -1.0 * predict_step_, -100.0 / 57.3 * predict_step_;
    du_max_ <<  1.0 * predict_step_,  100.0 / 57.3 * predict_step_;
}

void PlanningMPC::UpdateErrorModel()
{
    VectorXd x(nx_), xr(nx_);

    x << sensor_info_planner_.x, sensor_info_planner_.y,
         sensor_info_planner_.yaw, sensor_info_planner_.w;

    xr << local_ref_traj_point_.x,   local_ref_traj_point_.y,
          local_ref_traj_point_.yaw, local_ref_traj_point_.w;

    matrix_kesi_ << x - xr, u_pre_;

    double T1 = 0.07;

    matrix_a_ << 1.0, 0.0, -local_ref_traj_point_.v * predict_step_ * sin(local_ref_traj_point_.yaw),  0.0,
                 0.0, 1.0,  local_ref_traj_point_.v * predict_step_ * cos(local_ref_traj_point_.yaw),  0.0,
                 0.0, 0.0,  1.0,                                                                         predict_step_,
                 0.0, 0.0,  0.0,                                                                         1.0 - predict_step_ / T1;

    matrix_b_ << predict_step_ * cos(local_ref_traj_point_.yaw), 0.0,
                 predict_step_ * sin(local_ref_traj_point_.yaw), 0.0,
                 0.0,                                            0.0,
                 0.0,                                            predict_step_ / T1;

    p_savedata_->file << " [plannin_tracking_error] "
                      << " Time "    << sensor_info_.t
                      << " err_x "   << matrix_kesi_(0)
                      << " err_y "   << matrix_kesi_(1)
                      << " err_yaw " << matrix_kesi_(2)
                      << " t "       << sensor_info_planner_.t
                      << endl;
}

void PlanningMPC::UpdateIncrementModel()
{
    matrix_A_.setZero(nx_ + nu_, nx_ + nu_);
    matrix_A_.block(0, 0, nx_, nx_) = matrix_a_;
    matrix_A_.block(0, nx_, nx_, nu_) = matrix_b_;
    matrix_A_.block(nx_, nx_, nu_, nu_) = MatrixXd::Identity(nu_, nu_);

    matrix_B_.setZero(nx_ + nu_, nu_);
    matrix_B_.block(0, 0, nx_, nu_) = matrix_b_;
    matrix_B_.block(nx_, 0, nu_, nu_) = MatrixXd::Identity(nu_, nu_);

    matrix_C_.setZero(nx_, nx_ + nu_);
    matrix_C_.block(0, 0, nx_, nx_) = MatrixXd::Identity(nx_, nx_);
}

void PlanningMPC::Predict(MatrixXd &matrix_phi, MatrixXd &matrix_theta)
{
    matrix_phi.setZero(nx_ * np_, nx_ + nu_);

    for (int i = 0; i < np_; i++) {
        MatrixXd matrix_A_i = 
                MatrixXd::Identity(matrix_A_.rows(), matrix_A_.cols());

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

void PlanningMPC::CalObsCostFunc(
        MatrixXd &matrix_h_obs, MatrixXd &matrix_g_obs,
        MatrixXd matrix_phi, MatrixXd matrix_theta)
{
    FindNearestObsPos(sensor_info_planner_.x, sensor_info_planner_.y);

    double k, v;

    k = 1.0;
    v = sensor_info_planner_.v;
    if (v >= 0.0 && v < 0.05) {
        v = 0.05;
    } else if (v < 0.0 && v > -0.05) {
        v = -0.05;
    }

    double dis2obs_x, dis2obs_y, dis2obs_square, weight_obs;

    dis2obs_x = sensor_info_planner_.x - obs_x_near_.at(0);
    dis2obs_y = sensor_info_planner_.y - obs_y_near_.at(0);
    dis2obs_square = (pow(dis2obs_x, 2.0) + pow(dis2obs_y, 2.0)) + 1.0 - dis2obs_min_;
    weight_obs = k * v;

    double dJ_dx, dJ_dy, ddJ_ddx, ddJ_ddy, ddJ_dxdy;

    dJ_dx = -2.0 * weight_obs / dis2obs_square * (dis2obs_x);
    dJ_dy = -2.0 * weight_obs / dis2obs_square * (dis2obs_y);

    ddJ_ddx =
            4.0 * weight_obs / pow(dis2obs_square, 2.0) *
            pow(dis2obs_x, 2.0) - 2.0 * weight_obs / dis2obs_square;
    ddJ_ddy =
            4.0 * weight_obs / pow(dis2obs_square, 2.0) *
            pow(dis2obs_y, 2.0) - 2.0 * weight_obs / dis2obs_square;

    ddJ_dxdy =
            4.0 * weight_obs / pow(dis2obs_square, 2.0) *
            dis2obs_x * dis2obs_y;

    MatrixXd matrix_Ak(1, nx_), matrix_Hk(nx_, nx_);

    matrix_Ak << dJ_dx,    dJ_dy,    0.0, 0.0;
    matrix_Hk << ddJ_ddx,  ddJ_dxdy, 0.0, 0.0,
                 ddJ_dxdy, ddJ_ddy,  0.0, 0.0,
                 0.0,      0.0,      0.0, 0.0,
                 0.0,      0.0,      0.0, 0.0;

    MatrixXd matrix_N(1, np_ * nx_), matrix_M(np_ * nx_, np_ * nx_);

    matrix_N =
            CustomFunction::KroneckerProduct(MatrixXd::Ones(1, np_),
                                             matrix_Ak);
    matrix_M =
            CustomFunction::KroneckerProduct(MatrixXd::Identity(np_, np_),
                                             matrix_Hk);

    matrix_h_obs = matrix_theta.transpose() * matrix_M * matrix_theta;
    matrix_h_obs = (matrix_h_obs + matrix_h_obs.transpose()) * 0.5;

    MatrixXd matrix_e(np_ * nx_, 1);
    matrix_e = matrix_phi * matrix_kesi_;

    matrix_g_obs =
            (matrix_N * matrix_theta +
            (matrix_e.transpose()) * matrix_M * matrix_theta).transpose();
}

void PlanningMPC::CalObjectiveFunc(
        MatrixXd &matrix_h, MatrixXd &matrix_e, VectorXd &matrix_g,
        MatrixXd matrix_phi, MatrixXd matrix_theta)
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

    matrix_e = matrix_phi * matrix_kesi_;
    matrix_g = ((matrix_e.transpose()) * matrix_Q * matrix_theta).transpose();

    MatrixXd matrix_h_obs, matrix_g_obs;
    CalObsCostFunc(matrix_h_obs, matrix_g_obs, matrix_phi, matrix_theta);

    /* matrix_h += matrix_h_obs;
    matrix_g += matrix_g_obs; */

    static int gate = 0;
    if (gate % 40 == 0) {
        cout << " x: " << sensor_info_planner_.x
             << " y: " << sensor_info_planner_.y << endl << endl;

        cout << "matrix_g:" << endl << matrix_g
             << endl << endl << endl;

        cout << "matrix_g_obs:" << endl << matrix_g_obs
             << endl << endl << endl;

        cout << "matrix_h:" << endl << matrix_h
             << endl << endl << endl;

        cout << "matrix_h_obs:" << endl << matrix_h_obs
             << endl << endl << endl;
    }
    gate++;
}

void PlanningMPC::CalConstraintConditions(
        MatrixXd &matrix_A, VectorXd &lb, VectorXd &ub)
{
    MatrixXd matrix_A_t = MatrixXd::Zero(nc_, nc_);

    for (int i = 0; i < nc_; i++) {
        for (int j = 0; j <= i; j++) {
            matrix_A_t(i, j) = 1;
        }
    }

    MatrixXd matrix_matrix_A_i(nc_ * nu_, nc_ * nu_);

    matrix_matrix_A_i =
            CustomFunction::KroneckerProduct(matrix_A_t,
                                             MatrixXd::Identity(nu_, nu_));    

    MatrixXd Ut(nc_ * nu_, 1);
    Ut = CustomFunction::KroneckerProduct(MatrixXd::Ones(nc_, 1), u_pre_);

    MatrixXd Umin(nc_ * nu_, 1), Umax(nc_ * nu_, 1);

    Umin = CustomFunction::KroneckerProduct(MatrixXd::Ones(nc_, 1), u_min_);

    Umax = CustomFunction::KroneckerProduct(MatrixXd::Ones(nc_, 1), u_max_);

    MatrixXd delta_Umin(nc_ * nu_, 1), delta_Umax(nc_ * nu_, 1);

    delta_Umin =
            CustomFunction::KroneckerProduct(MatrixXd::Ones(nc_, 1), du_min_);

    delta_Umax =
            CustomFunction::KroneckerProduct(MatrixXd::Ones(nc_, 1), du_max_);

    /* dUmin(0) * call_cycle_ / predict_step_*/
    for (int i = 0; i < nu_; i++) {
        delta_Umin(i, 0) = delta_Umin(i, 0) * call_cycle_ / predict_step_;
        delta_Umax(i, 0) = delta_Umax(i, 0) * call_cycle_ / predict_step_;
    }

    matrix_A.block(0, 0, nc_ * nu_, nc_ * nu_) = matrix_matrix_A_i;

    matrix_A.block(nc_ * nu_, 0, nc_ * nu_, nc_ * nu_) =
            MatrixXd::Identity(nc_ * nu_, nc_ * nu_);

    lb << Umin - Ut, delta_Umin;

    ub << Umax - Ut, delta_Umax;
}

int PlanningMPC::OptimizationSolver(
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

    c_int m = matrix_Ac.rows();
    c_int n = matrix_p.cols();

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

void PlanningMPC::MatrixToCCS(
        MatrixXd matrix_, vector<c_float> *sm_x, c_int &sm_nnz,
        vector<c_int> *sm_i, vector<c_int> *sm_p)
{
    sm_p->emplace_back(0);

    int cols = matrix_.cols(), rows = matrix_.rows(), nz = 0;

    if (cols == rows) {
        for( int j = 0; j < cols; j++) {
            for (int i = 0; i <= j; i++) {
                if(fabs(matrix_(i,j))>0.0000001) {
                    sm_x->emplace_back(matrix_(i, j));
                    sm_i->emplace_back(i);

                    nz++;
                }
            }

            sm_p->emplace_back(nz);
        }
    } else if (cols < rows) {
        for( int j = 0; j < cols; j++) {
            for (int i = 0; i < rows; i++) {
                if(fabs(matrix_(i,j))>0.0000001) {
                    sm_x->emplace_back(matrix_(i, j));
                    sm_i->emplace_back(i);

                    nz++;
                }
            }

            sm_p->emplace_back(nz);
        }
    }

    sm_nnz = nz;
}

void PlanningMPC::UpdateReferenceTrajectory()
{
    RobotMotionStatePara motion_state =
            {sensor_info_planner_.x, sensor_info_planner_.y,
             sensor_info_planner_.yaw, sensor_info_planner_.v,
             sensor_info_planner_.w, sensor_info_planner_.t};

    opt_traj_points_.resize(np_ + 1);

    /* Todo: 最少预测np_个节点，当曲率较大时，增加预测节点的数量（以转弯半径、小车速度，
          动态调整预测步长），从而规避曲线拟合
    Todo: 以上设计思路需与控制方案相适应，体现规划与控制的耦合性与交互性
    Todo: 控制算法求参考点的位置、速度、角速度算法需要优化，单一的时间线性插值，
          改为推算速度、位置的“线性”插值，且先推算速度、再求解位置 */
    for (int i = 0; i <= np_; i++) {
        /* motion_state.v = u_optimal_(0);
        motion_state.w = u_optimal_(1); */

        if (start_gate_ == false) {
            if (i >= 1) {
                ControlCommand control_command;

                if (i < nc_) {
                    control_command.speed_command = u_optimal_(nu_ * i); // patch
                    control_command.yaw_rate_command =
                            u_optimal_(nu_ * i + 1);
                } else {
                    control_command.speed_command = u_optimal_(nu_ * nc_ - nu_);
                    control_command.yaw_rate_command =
                            u_optimal_(nu_ * nc_ + 1 - nu_);
                }

                double Tv = 0.3;
                double acceleration =
                        (control_command.speed_command - motion_state.v) / Tv;

                double Tw = 0.3;
                double yaw_acceleration =
                        (control_command.yaw_rate_command -
                        motion_state.w) / Tw;

                double simulation_step = predict_step_;

                motion_state.v = motion_state.v + acceleration*simulation_step;
                motion_state.w = 
                        motion_state.w + yaw_acceleration*simulation_step;

                motion_state.yaw =
                        motion_state.yaw + motion_state.w * simulation_step;

                motion_state.x =
                        motion_state.x + motion_state.v *
                        cos(motion_state.yaw) * simulation_step;
                motion_state.y =
                        motion_state.y + motion_state.v *
                        sin(motion_state.yaw) * simulation_step;

                motion_state.t = motion_state.t + simulation_step;
            } else {
                motion_state.v = u_optimal_(0);  // patch
                motion_state.w = u_optimal_(1);
            }

            opt_traj_points_.at(i).t_ref   = motion_state.t;
            opt_traj_points_.at(i).x_ref   = motion_state.x;
            opt_traj_points_.at(i).y_ref   = motion_state.y;
            opt_traj_points_.at(i).yaw_ref = motion_state.yaw;
            opt_traj_points_.at(i).v_ref   = motion_state.v;
            opt_traj_points_.at(i).w_ref   = motion_state.w;
        } else {
            if (i > weak_planning_num_) {
                ControlCommand control_command;

                if (i - weak_planning_num_ <= nc_) {
                    control_command.speed_command =
                            u_optimal_(nu_ * (i - weak_planning_num_) - nu_);

                    control_command.yaw_rate_command =
                            u_optimal_(nu_ * (i - weak_planning_num_)
                            + 1 - nu_);
                } else {
                    control_command.speed_command =
                            u_optimal_(nu_ * nc_ - nu_);
                            
                    control_command.yaw_rate_command =
                            u_optimal_(nu_ * nc_ + 1 - nu_);
                }

                double Tv = 0.1;
                double acceleration =
                        (control_command.speed_command - motion_state.v) / Tv;

                double Tw = 0.1;
                double yaw_acceleration =
                        (control_command.yaw_rate_command -
                        motion_state.w) / Tw;
                                                                
                double simulation_step = predict_step_;

                motion_state.v = motion_state.v + acceleration*simulation_step;
                motion_state.w = 
                        motion_state.w + yaw_acceleration*simulation_step;

                motion_state.yaw =
                        motion_state.yaw + motion_state.w * simulation_step;

                motion_state.x =
                        motion_state.x + motion_state.v *
                        cos(motion_state.yaw) * simulation_step;
                motion_state.y =
                        motion_state.y + motion_state.v *
                        sin(motion_state.yaw) * simulation_step;

                motion_state.t = motion_state.t + simulation_step;

                opt_traj_points_.at(i).t_ref   = motion_state.t;
                opt_traj_points_.at(i).x_ref   = motion_state.x;
                opt_traj_points_.at(i).y_ref   = motion_state.y;
                opt_traj_points_.at(i).yaw_ref = motion_state.yaw;
                opt_traj_points_.at(i).v_ref   = motion_state.v;
                opt_traj_points_.at(i).w_ref   = motion_state.w;
            } else {
                double index = i + planner_sensor_info_id_;

                opt_traj_points_.at(i).t_ref =
                        opt_traj_points_.at(index).t_ref;
                opt_traj_points_.at(i).x_ref =
                        opt_traj_points_.at(index).x_ref;
                opt_traj_points_.at(i).y_ref =
                        opt_traj_points_.at(index).y_ref;
                opt_traj_points_.at(i).yaw_ref =
                        opt_traj_points_.at(index).yaw_ref;
                opt_traj_points_.at(i).v_ref =
                        opt_traj_points_.at(index).v_ref;
                opt_traj_points_.at(i).w_ref =
                        opt_traj_points_.at(index).w_ref;
            }
        }

        p_savedata_->file << "[reference_trajectory_planning] "
                          << "Time "         << sensor_info_.t
                          << "x_ref "        << opt_traj_points_.at(i).x_ref
                          << "y_ref "        << opt_traj_points_.at(i).y_ref
                          << "yaw_ref "      << opt_traj_points_.at(i).yaw_ref
                          << "v_ref "        << opt_traj_points_.at(i).v_ref
                          << "w_ref "        << opt_traj_points_.at(i).w_ref
                          << "t_ref "        << opt_traj_points_.at(i).t_ref
                          << "loop_counter " << (double)loop_counter_ << endl;
    }

    start_gate_ = true;
}

void PlanningMPC::CalPredictForwardCommand()
{
    int size_ref_traj = local_traj_points_.size();

    double time_predict = sensor_info_planner_.t;

    double vc_ref, wc_ref;

    for (int i = 0; i < nc_; i++) {
        vector<double> relative_time;
        vector<double> fabs_relative_time;

        for (int i = 0; i < size_ref_traj; i++) {
            double delta_t, fabs_delta_t;

            delta_t = time_predict - local_traj_points_.at(i).t_ref;
            fabs_delta_t = -1.0 * fabs(delta_t);

            relative_time.push_back(delta_t);
            fabs_relative_time.push_back(fabs_delta_t);
        }

        vector<double>::iterator biggest =
                max_element(begin(fabs_relative_time),
                            end(fabs_relative_time));

        int ref_point_index;

        ref_point_index = distance(begin(fabs_relative_time), biggest);

        if (relative_time.at(ref_point_index) <= 0.0) {
            ref_point_index = ref_point_index - 1;

            if(ref_point_index == -1) {
                ref_point_index = 0;
            }
        } else {
            ref_point_index = ref_point_index;

            if (ref_point_index == (size_ref_traj - 1)) {
                ref_point_index = size_ref_traj - 2;
            }
        }

        double dis1, dis2, dis_total;

        dis1 = fabs_relative_time.at(ref_point_index);
        dis2 = fabs_relative_time.at(ref_point_index + 1);

        dis_total = dis1 + dis2;

        vc_ref =
                ( local_traj_points_.at(ref_point_index).v_ref * dis2 +
                local_traj_points_.at(ref_point_index + 1).v_ref * dis1 ) /
                dis_total;
        wc_ref =
                ( local_traj_points_.at(ref_point_index).w_ref * dis2 +
                local_traj_points_.at(ref_point_index + 1).w_ref * dis1 ) /
                dis_total;

        /* static int gate = 0;
        if (gate % 20 == 0) {
            cout << " time " << sensor_info_planner_.t
                 << " ref_point_index " << ref_point_index
                 << " vc_ref " << vc_ref << endl;
        }
        gate++; */

        time_predict += predict_step_;

        ref_point_command_.at(i * nu_) = vc_ref;
        ref_point_command_.at(i * nu_ + 1) = wc_ref;
    }
}

double PlanningMPC::GetRunningTimeAverage()
{
    return running_time_average_;
}
