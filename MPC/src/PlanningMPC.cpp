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

PlanningMPC::PlanningMPC(
        RobotModel *p_robot_model, SaveData *p_savedata, GoalState goal_state):
        PlanningAlgorithm::PlanningAlgorithm(
                p_robot_model, p_savedata, goal_state)
{
    start_gate_ = false;

    weak_planning_num_ = 3;

    weak_planning_duration_ = 0.16;

    call_cycle_ = 0.02;

    nx_ = 4;
    nu_ = 2;

    x_.resize(nx_ + nu_);
    a_.resize(nx_, nx_);
    b_.resize(nx_, nu_);

    u_max_.resize(nu_, 1);
    u_min_.resize(nu_, 1);
    du_max_.resize(nu_, 1);
    du_min_.resize(nu_, 1);

    u_pre_.resize(nu_);

    GetSensorInfo();

    //ReadInGoalTraj();
    GenerateGoalTraj();

    FindRefPoint();

    u_pre_ << sensor_info_.v - global_ref_traj_point_.v,
              sensor_info_.w - global_ref_traj_point_.w;

    running_time_sum_ = 0.0;
    running_time_average_ = 0.0;

    loop_counter_ = 0;

    gate_start_ = true;

    A_.resize(nx_ + nu_, nx_ + nu_);
    B_.resize(nx_ + nu_, nu_);
    C_.resize(nx_, nx_ + nu_);

    CalControlCoefficient();

    ref_point_command_.resize(nc_ * nu_);
}

//Todo
/* void PlanningMPC::CalRefTrajectory() */
ControlCommand PlanningMPC::CalRefTrajectory(
        vector<TrajPoint> &local_traj_points)
{
    struct timeval t_start, t_end;
    gettimeofday(&t_start,NULL);

    if (start_gate_ == false) {
        GetSensorInfo();
    } else {
        UpdateSensorInfo();
    }

    FindRefPoint();

    VectorXd u_min(nu_, 1), u_max(nu_, 1), du_min(nu_, 1), du_max(nu_, 1);
    MatrixXd q(nx_, nx_), r(nu_, nu_);

    CalControlCoefficient(); /* CalControlCoefficient(sensor_info_.v); */

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
    VectorXd u_opt(nc_ * nu_);

    c_int max_iteration = 200;
    c_float eps_abs = 0.01;

    OptimizationSolver(u_opt, H, g, _A, lb, ub, num_constraints,
                       num_variables, max_iteration, eps_abs);

    u_optimal_.resize(nc_ * nu_, 1);
    u_opt_storage_.resize(nc_ * nu_, 1);

    MatrixXd A_t = MatrixXd::Zero(nc_, nc_);

    for (int i = 0; i < nc_; i++) {
        for (int j = 0; j <= i; j++) {
            A_t(i, j) = 1;
        }
    }

    MatrixXd A_I(nc_ * nu_, nc_ * nu_);

    A_I = CustomFunction::KroneckerProduct(A_t, MatrixXd::Identity(nu_, nu_));

    MatrixXd Ut(nc_ * nu_, 1);
    Ut = CustomFunction::KroneckerProduct(MatrixXd::Ones(nc_, 1), u_pre_);

    u_optimal_ = Ut + A_I * u_opt;

    u_opt_storage_ = u_optimal_;

    u_pre_(0) = u_optimal_(0);
    u_pre_(1) = u_optimal_(1);

    /* Todo: 参考FindRefPoint函数 */
    VectorXd ref_point_command;

    ref_point_command.resize(nc_ * nu_);

    //线性化模型+osqp库的缺陷在这里，参考轨迹的速度、角速度变化时，预测轨迹误差较大，但不影响避障可行性
    CalPredictForwardCommand();

    for (int i = 0; i < nc_; i++) {
        for (int j = 0; j < nu_; j++) {
            u_optimal_(nu_ * i + j) =
                    u_optimal_(nu_ * i + j) + ref_point_command_.at(nu_ * i + j);
        }
    }

    UpdateReferenceTrajectory();

    local_traj_points.assign(loacl_trajectory_points_.begin(),
                             loacl_trajectory_points_.end());

    gettimeofday(&t_end, NULL);

    loop_counter_++;

    running_time_sum_ = running_time_sum_ + (t_end.tv_sec - t_start.tv_sec) +
                        (double)(t_end.tv_usec - t_start.tv_usec) / 1000000.0;

    running_time_average_ = running_time_sum_ / (double)loop_counter_;

    p_savedata_->file << "[planning_control_command] "
                          << " Time "             << sensor_info_.t
                          << " vc "               << u_optimal_(0)
                          << " wc "               << u_optimal_(1)
                          << " Time "             << sensor_info_.t << endl;

    ControlCommand result = {u_optimal_(0), u_optimal_(1), sensor_info_.t};

    return result;
}

void PlanningMPC::ReadInGoalTraj()
{
    string string_temp;

    ifstream ReadFile;
    ReadFile.open("../data/TrajectoryPoints.txt", ios::in);

    if (ReadFile.fail()) {
        cout << "[error] failed to open TrajectoryPoints.txt" << endl;
    } else {
        while (getline(ReadFile, string_temp)) {
            istringstream is(string_temp);

            double data;

            vector<double> Temp;
            TrajPoint temp;

            while (!is.eof()) {
                is>>data;
                Temp.push_back(data);
            }

            temp.x_ref   = Temp.at(0);
            temp.y_ref   = Temp.at(1);
            temp.yaw_ref = Temp.at(2);
            temp.v_ref   = Temp.at(3);
            temp.w_ref   = Temp.at(4);
            temp.t_ref   = Temp.at(5);

            global_traj_points_.push_back(temp);

            Temp.clear();
            string_temp.clear();
        }
    }

    ReadFile.close();
    cout << "[INFO] read in reference global route points successfully !"
         << endl;
}

void PlanningMPC::GenerateGoalTraj()
{
    double distance_agv2goal = pow(pow(goal_state_.x - sensor_info_.x, 2.0) + 
                                     pow(goal_state_.y - sensor_info_.y, 2.0), 0.5);

    double min_relative_dis = 1.0;

    if (distance_agv2goal < min_relative_dis) {
        cout << "[error] agv is too near to goal point!" << endl;
    }

    TrajPoint temp;

    temp.x_ref   = sensor_info_.x;
    temp.y_ref   = sensor_info_.y;
    temp.yaw_ref = sensor_info_.yaw;
    temp.v_ref   = sensor_info_.v;
    temp.w_ref   = sensor_info_.w;
    temp.t_ref   = sensor_info_.t;

    global_traj_points_.push_back(temp);

    double step = 0.05;

    double distance_travel;

    double min_speed = 0.01;

    double acc = 0.0;

    if (fabs(sensor_info_.v) < min_speed) {
        acc = 0.1;

        double delta_time = (min_speed - sensor_info_.v) / acc;

        distance_travel = 
                sensor_info_.v * delta_time + 0.5 * acc * pow(delta_time, 2.0);

        temp.x_ref   = sensor_info_.x + distance_travel * cos(temp.yaw_ref);
        temp.y_ref   = sensor_info_.y + distance_travel * sin(temp.yaw_ref);
        temp.yaw_ref = sensor_info_.yaw + sensor_info_.w * delta_time;
        temp.v_ref   = min_speed;
        temp.w_ref   = sensor_info_.w;
        temp.t_ref   = sensor_info_.t + delta_time;

        global_traj_points_.push_back(temp);
    }

    // Todo 代码风格
    // Todo 对时间t1、速度speed_except采样，形成备选轨迹集合
    // Todo 明确场景与边界条件，建模仿真
    // Todo 调参，分析仿真数据，确定代价函数和硬件约束
    // Todo 调试，定版
    double speed_except = 0.15;

    double tiem_margin = 1.0;

    double safe_distance = 0.3; // Todo 随速度自适应A

    double t0, t1;
    
    t0 = temp.t_ref;
    t1 = 
            t0 + (distance_agv2goal - safe_distance - distance_travel) /
            speed_except + tiem_margin;

    cout << t1 - t0 << endl;

    MatrixXd X(6, 1), Y(6, 1);
    
    X << temp.x_ref,
         temp.v_ref * cos(temp.yaw_ref),
         acc * cos(temp.yaw_ref),
         goal_state_.x - safe_distance * cos(goal_state_.yaw),
         speed_except * cos(goal_state_.yaw),
         0.0;    

    Y << temp.y_ref,
         temp.v_ref * sin(temp.yaw_ref),
         acc * sin(temp.yaw_ref),
         goal_state_.y - safe_distance * sin(goal_state_.yaw),
         speed_except * sin(goal_state_.yaw),
         0.0;

    MatrixXd T(6, 6);
    
    T << pow(t0, 5.0),         pow(t0, 4.0),         pow(t0, 3.0),        pow(t0, 2.0),  t0,   1.0,
         5.0 * pow(t0, 4.0),   4.0 * pow(t0, 3.0),   3.0 * pow(t0, 2.0),  2.0 * t0,      1.0,  0.0,
         20.0 * pow(t0, 3.0),  12.0 * pow(t0, 2.0),  6.0 * t0,            2.0,           0.0,  0.0,
         pow(t1, 5.0),         pow(t1, 4.0),         pow(t1, 3.0),        pow(t1, 2.0),  t1,   1.0,
         5.0 * pow(t1, 4.0),   4.0 * pow(t1, 3.0),   3.0 * pow(t1, 2.0),  2.0 * t1,      1.0,  0.0,
         20.0 * pow(t1, 3.0),  12.0 * pow(t1, 2.0),  6.0 * t1,            2.0,           0.0,  0.0;

    MatrixXd A(6, 1), B(6, 1);

    A = T.inverse() * X;

    B = T.inverse() * Y;

    int cycle_num = (int)((t1 - t0) / step) + 1;

    double yaw = sensor_info_.yaw;

    for (int i = 1; i < cycle_num; i++) {
        double t = t0 + i * step;

        MatrixXd matrix_T(3, 6);

        matrix_T << pow(t, 5.0),         pow(t, 4.0),         pow(t, 3.0),        pow(t, 2.0),  t,   1.0,
                    5.0 * pow(t, 4.0),   4.0 * pow(t, 3.0),   3.0 * pow(t, 2.0),  2.0 * t,      1.0,  0.0,
                    20.0 * pow(t, 3.0),  12.0 * pow(t, 2.0),  6.0 * t,            2.0,           0.0,  0.0;

        MatrixXd result_x(3, 1), result_y(3, 1);

        result_x = matrix_T * A;

        result_y = matrix_T * B;

        double x, vx, ax, y, vy, ay;

        x = result_x(0);
        vx = result_x(1);
        ax = result_x(2);

        y = result_y(0);
        vy = result_y(1);
        ay = result_y(2);

        double v, curvature, w;

        v = pow(vx * vx + vy * vy, 0.5);

        yaw = atan2(vy, vx);

        if (v != 0) {
            curvature =
                    (vx * ay - vy * ax) /
                    pow(pow(vx, 2.0) + pow(vy, 2.0), 3.0 / 2.0);

            w = v * curvature;
        } else {
            curvature = 9999999999.9999999;

            w = 0.0;
        }

        temp.x_ref   = x;
        temp.y_ref   = y;
        temp.yaw_ref = yaw;
        temp.v_ref   = v;
        temp.w_ref   = w;
        temp.t_ref   = t;

        global_traj_points_.push_back(temp);
    }

    temp.x_ref   = goal_state_.x;
    temp.y_ref   = goal_state_.y;
    temp.yaw_ref = goal_state_.yaw;
    temp.v_ref   = goal_state_.v + speed_except;
    temp.w_ref   = goal_state_.w;
    temp.t_ref   = temp.t_ref + safe_distance / speed_except;

    global_traj_points_.push_back(temp);

    cout << "[INFO] generate reference global route points successfully !"
         << endl;
}

void PlanningMPC::FindRefPoint()
{
    int SizeOfRefTraj = global_traj_points_.size();

    if (SizeOfRefTraj <= 1 ) {
        cout << "[error] global reference trajectory has only "
             << SizeOfRefTraj << " points " << endl << endl;
    }

    vector<double> RelativeTime;
    vector<double> fabs_RelativeTime;

    for (int i = 0; i < SizeOfRefTraj; i++) {
        double delta_t, fabs_delta_t;

        delta_t = sensor_info_.t - global_traj_points_.at(i).t_ref;
        fabs_delta_t = -1.0 * fabs(delta_t);

        RelativeTime.push_back(delta_t);
        fabs_RelativeTime.push_back(fabs_delta_t);
    }

    vector<double>::iterator biggest =
            max_element(begin(fabs_RelativeTime), end(fabs_RelativeTime));

    int _ID, ID_RefPoint;

    _ID = distance(begin(fabs_RelativeTime), biggest);

    if (RelativeTime.at(_ID) <= 0.0) {
        ID_RefPoint = _ID - 1;

        if(ID_RefPoint == -1) {
            ID_RefPoint = 0;
        }
    } else {
        ID_RefPoint = _ID;

        if (ID_RefPoint == (SizeOfRefTraj - 1)) {
            ID_RefPoint = SizeOfRefTraj - 2;
        }
    }

    double dis1, dis2, dis_total;

    dis1 = fabs_RelativeTime.at(ID_RefPoint);
    dis2 = fabs_RelativeTime.at(ID_RefPoint + 1);

    dis_total = dis1 + dis2;

    global_ref_traj_point_.x =
            ( global_traj_points_.at(ID_RefPoint).x_ref * dis2 +
              global_traj_points_.at(ID_RefPoint + 1).x_ref * dis1 ) /
              dis_total;
    global_ref_traj_point_.y =
            ( global_traj_points_.at(ID_RefPoint).y_ref * dis2 +
              global_traj_points_.at(ID_RefPoint + 1).y_ref * dis1 ) /
              dis_total;
    global_ref_traj_point_.yaw =
            ( global_traj_points_.at(ID_RefPoint).yaw_ref * dis2 +
              global_traj_points_.at(ID_RefPoint + 1).yaw_ref * dis1 ) /
              dis_total;
    global_ref_traj_point_.v =
            ( global_traj_points_.at(ID_RefPoint).v_ref * dis2 +
              global_traj_points_.at(ID_RefPoint + 1).v_ref * dis1 ) /
            dis_total;
    global_ref_traj_point_.w =
            ( global_traj_points_.at(ID_RefPoint).w_ref * dis2 +
              global_traj_points_.at(ID_RefPoint + 1).w_ref * dis1 ) /
              dis_total;
    global_ref_traj_point_.t =
            ( global_traj_points_.at(ID_RefPoint).t_ref * dis2 +
              global_traj_points_.at(ID_RefPoint + 1).t_ref * dis1 ) /
              dis_total;
         
    p_savedata_->file << "[plainning_global_reference_point] "
                      << " Time "    << global_ref_traj_point_.t
                      << " x_ref "   << global_ref_traj_point_.x
                      << " y_ref "   << global_ref_traj_point_.y
                      << " yaw_ref " << global_ref_traj_point_.yaw
                      << " v_ref "   << global_ref_traj_point_.v
                      << " w_ref "   << global_ref_traj_point_.w
                      << " t "       << global_ref_traj_point_.t << endl;
}

void PlanningMPC::UpdateSensorInfo()
{
    int SizeOfRefTraj = ref_traj_points_.size();

    if (SizeOfRefTraj <= 1 ) {
        cout << "[error] global reference trajectory has only "
             << SizeOfRefTraj << " points " << endl << endl;
    }

    vector<double> RelativeTime;
    vector<double> fabs_RelativeTime;

    for (int i = 0; i < SizeOfRefTraj; i++) {
        double delta_t, fabs_delta_t;

        delta_t = p_robot_model_->motion_state_.t - ref_traj_points_.at(i).t_ref;
        fabs_delta_t = -1.0 * fabs(delta_t);

        RelativeTime.push_back(delta_t);
        fabs_RelativeTime.push_back(fabs_delta_t);
    }

    vector<double>::iterator biggest =
            max_element(begin(fabs_RelativeTime), end(fabs_RelativeTime));

    int _ID, ID_RefPoint;

    _ID = distance(begin(fabs_RelativeTime), biggest);

    if (RelativeTime.at(_ID) <= 0.0) {
        ID_RefPoint = _ID - 1;

        if(ID_RefPoint == -1) {
            ID_RefPoint = 0;
        }
    } else {
        ID_RefPoint = _ID;

        if (ID_RefPoint == (SizeOfRefTraj - 1)) {
            ID_RefPoint = SizeOfRefTraj - 2;
        }
    }

    int index_transition_point = ID_RefPoint + weak_planning_num_;
    
    sensor_info_.t   = ref_traj_points_.at(index_transition_point).t_ref;
    sensor_info_.x   = ref_traj_points_.at(index_transition_point).x_ref;
    sensor_info_.y   = ref_traj_points_.at(index_transition_point).y_ref;
    sensor_info_.yaw = ref_traj_points_.at(index_transition_point).yaw_ref;
    sensor_info_.v   = ref_traj_points_.at(index_transition_point).v_ref;
    sensor_info_.w   = ref_traj_points_.at(index_transition_point).w_ref;

    sensor_info_id_ = ID_RefPoint;

    // 控制量初始值求解方案A
    /* u_pre_ << sensor_info_.v - global_ref_traj_point_.v,
              sensor_info_.w - global_ref_traj_point_.w; */
    
    int index_init_point = index_transition_point - 1;

    // 控制量初始值求解方案B
    // 在规划轨迹的连续性上，方案B优于方案A，源于强、弱规划的衔接策略 + 运动学线性化误差模型
    // 方案B规划的轨迹更顺化，易于跟踪，小车对全局路径的跟踪精度也会更高
    u_pre_ << u_opt_storage_(index_init_point * nu_),
              u_opt_storage_(index_init_point * nu_ + 1);
}

void PlanningMPC::CalControlCoefficient()
{
    np_ = 15;
    nc_ = 6;

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

    predict_step_ = 0.08;

    // Todo： 无人车控制的约束项应扣除参考轨迹的速度、角速度、加速度、角加速度
    // Todo： 规划中的约束项应扣除各预测点的参考速度、角速度、加速度、角加速度
    // Todo： 约束项扣除的参开速度、角速度应随预测点调整，预测模型A应随预测点调整
    //  说明： 如果是非线性模型、非线性优化，则不需要以上处理
    u_min_ << -1.0 - global_ref_traj_point_.v,
              -35.0 / 57.3 - global_ref_traj_point_.w;

    u_max_ <<  1.0 - global_ref_traj_point_.v,
               35.0 / 57.3 - global_ref_traj_point_.w;

    // du_min_(0): * call_cycle_; du_min_(i)(i >= 1): * predict_step_
    du_min_ << -1.0 * predict_step_, -100.0 / 57.3 * predict_step_;
    du_max_ <<  1.0 * predict_step_,  100.0 / 57.3 * predict_step_;
}

void PlanningMPC::UpdateErrorModel()
{
    VectorXd x(nx_), xr(nx_), kesi(nx_ + nu_);

    x << sensor_info_.x, sensor_info_.y, sensor_info_.yaw, sensor_info_.w;

    xr << global_ref_traj_point_.x,   global_ref_traj_point_.y,
          global_ref_traj_point_.yaw, global_ref_traj_point_.w;

    kesi << x-xr, u_pre_;

    x_ = kesi;

    double T1 = 0.07;

    a_ << 1.0, 0.0, -global_ref_traj_point_.v*predict_step_*sin(global_ref_traj_point_.yaw),  0.0,
          0.0, 1.0,  global_ref_traj_point_.v*predict_step_*cos(global_ref_traj_point_.yaw),  0.0,
          0.0, 0.0,  1.0,                                                                     predict_step_,
          0.0, 0.0,  0.0,                                                                     1.0 - predict_step_ / T1;

    b_ << predict_step_ * cos(global_ref_traj_point_.yaw), 0.0,
          predict_step_ * sin(global_ref_traj_point_.yaw), 0.0,
          0.0,                                             0.0,
          0.0,                                             predict_step_ / T1;

    p_savedata_->file << " [plannin_tracking_error] "
                      << " Time "    << sensor_info_.t
                      << " err_x "   << x_(0)
                      << " err_y "   << x_(1)
                      << " err_yaw " << x_(2)
                      << " t "       << sensor_info_.t << endl;
}

void PlanningMPC::UpdateIncrementModel()
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

void PlanningMPC::PredictFunc(MatrixXd &phi, MatrixXd &theta)
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

void PlanningMPC::ObjectiveFunc(
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

void PlanningMPC::ConstraintCondition(MatrixXd &A, VectorXd &lb, VectorXd &ub)
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
    Ut = CustomFunction::KroneckerProduct(MatrixXd::Ones(nc_, 1), u_pre_);

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
        delta_Umin(i, 0) = delta_Umin(i, 0) * call_cycle_ / predict_step_;
        delta_Umax(i, 0) = delta_Umax(i, 0) * call_cycle_ / predict_step_;
    }

    A.block(0, 0, nc_ * nu_, nc_ * nu_) = A_I;

    A.block(nc_ * nu_, 0, nc_ * nu_, nc_ * nu_) =
            MatrixXd::Identity(nc_ * nu_, nc_ * nu_);

    lb << Umin - Ut, delta_Umin;

    ub << Umax - Ut, delta_Umax;
}

int PlanningMPC::OptimizationSolver(
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
                    sm_x->emplace_back(matrix_(i,j));
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

//Todo
void PlanningMPC::UpdateReferenceTrajectory()
{
    RobotMotionStatePara motion_state =
            {sensor_info_.x, sensor_info_.y, sensor_info_.yaw,
             sensor_info_.v, sensor_info_.w, sensor_info_.t};

    ref_traj_points_.resize(np_ + 1);

    // Todo: 最少预测np_个节点，当曲率较大时，增加预测节点的数量（以转弯半径、小车速度，动态调整预测步长），从而规避曲线拟合
    // Todo: 以上设计思路需与控制方案相适应，体现规划与控制的耦合性与交互性
    // Todo: 控制算法求参考点的位置、速度、角速度算法需要优化，单一的时间线性插值，改为推算速度、位置的“线性”插值，且先推算速度、再求解位置
    for (int i = 0; i <= np_; i++) {
        if (start_gate_ == false) {
            if (i >= 1) {
                ControlCommand control_command;

                if (i <= nc_) {
                    control_command.speed_command = u_optimal_(nu_ * i - nu_);
                    control_command.yaw_rate_command =
                            u_optimal_(nu_ * i + 1 - nu_);
                } else {
                    control_command.speed_command = u_optimal_(nu_ * nc_ - nu_);
                    control_command.yaw_rate_command =
                            u_optimal_(nu_ * nc_ + 1 - nu_);
                }

                double Tv = 0.1;
                double acceleration =
                        (control_command.speed_command - motion_state.v) / Tv;

                double Tw = 0.15;
                double yaw_acceleration =
                        (control_command.yaw_rate_command - motion_state.w) / Tw;

                double simulation_step = predict_step_;

                /* motion_state.v = control_command.speed_command;
                motion_state.w = control_command.yaw_rate_command; */
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
            }

            ref_traj_points_.at(i).t_ref   = motion_state.t;
            ref_traj_points_.at(i).x_ref   = motion_state.x;
            ref_traj_points_.at(i).y_ref   = motion_state.y;
            ref_traj_points_.at(i).yaw_ref = motion_state.yaw;
            ref_traj_points_.at(i).v_ref   = motion_state.v;
            ref_traj_points_.at(i).w_ref   = motion_state.w;
        } else {
            if (i > weak_planning_num_) {
                ControlCommand control_command;

                if (i - weak_planning_num_ <= nc_) {
                    control_command.speed_command =
                            u_optimal_(nu_ * (i - weak_planning_num_) - nu_);

                    control_command.yaw_rate_command =
                            u_optimal_(nu_ * (i - weak_planning_num_) + 1 - nu_);
                } else {
                    control_command.speed_command = u_optimal_(nu_ * nc_ - nu_);
                    control_command.yaw_rate_command =
                            u_optimal_(nu_ * nc_ + 1 - nu_);
                }

                double Tv = 0.1;
                double acceleration =
                        (control_command.speed_command - motion_state.v) / Tv;

                double Tw = 0.15;
                double yaw_acceleration =
                        (control_command.yaw_rate_command - motion_state.w) / Tw;

                double simulation_step = predict_step_;

                /* motion_state.v = control_command.speed_command;
                motion_state.w = control_command.yaw_rate_command; */
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

                ref_traj_points_.at(i).t_ref   = motion_state.t;
                ref_traj_points_.at(i).x_ref   = motion_state.x;
                ref_traj_points_.at(i).y_ref   = motion_state.y;
                ref_traj_points_.at(i).yaw_ref = motion_state.yaw;
                ref_traj_points_.at(i).v_ref   = motion_state.v;
                ref_traj_points_.at(i).w_ref   = motion_state.w;
            } else {
                ref_traj_points_.at(i).t_ref =
                        ref_traj_points_.at(i + sensor_info_id_).t_ref;
                ref_traj_points_.at(i).x_ref =
                        ref_traj_points_.at(i + sensor_info_id_).x_ref;
                ref_traj_points_.at(i).y_ref =
                        ref_traj_points_.at(i + sensor_info_id_).y_ref;
                ref_traj_points_.at(i).yaw_ref =
                        ref_traj_points_.at(i + sensor_info_id_).yaw_ref;
                ref_traj_points_.at(i).v_ref =
                        ref_traj_points_.at(i + sensor_info_id_).v_ref;
                ref_traj_points_.at(i).w_ref =
                        ref_traj_points_.at(i + sensor_info_id_).w_ref;
            }
        }

        p_savedata_->file << "[reference_trajectory_planning] "
                          << " Time "           << sensor_info_.t
                          << " x_ref "         << ref_traj_points_.at(i).x_ref
                          << " y_ref "         << ref_traj_points_.at(i).y_ref
                          << " yaw_ref "       << ref_traj_points_.at(i).yaw_ref
                          << " v_ref "         << ref_traj_points_.at(i).v_ref
                          << " w_ref "         << ref_traj_points_.at(i).w_ref
                          << " t_ref "         << ref_traj_points_.at(i).t_ref
                          << " loop_counter "  << (double)loop_counter_ << endl;
    }

    loacl_trajectory_points_.assign(
            ref_traj_points_.begin(), ref_traj_points_.end());

    start_gate_ = true;

    p_savedata_->file << "[goal_state_planning] "
                          << " Time "         << sensor_info_.t
                          << " x_goal "       << goal_state_.x
                          << " y_goal "       << goal_state_.y
                          << " yaw_goal "     << goal_state_.yaw
                          << " v_goal "       << goal_state_.v
                          << " w_goal "       << goal_state_.w
                          << " Time "         << sensor_info_.t
                          << " loop_counter " << (double)loop_counter_<< endl;
}

//Todo
void PlanningMPC::SmoothTrajcecory()
{
    ;
}

void PlanningMPC::CalPredictForwardCommand()
{
    int SizeOfRefTraj = global_traj_points_.size();

    double time_predict = sensor_info_.t;

    double vc_ref, wc_ref;

    for (int i = 0; i < nc_; i++) {
        vector<double> RelativeTime;
        vector<double> fabs_RelativeTime;

        for (int i = 0; i < SizeOfRefTraj; i++) {
            double delta_t, fabs_delta_t;

            delta_t = time_predict - global_traj_points_.at(i).t_ref;
            fabs_delta_t = -1.0 * fabs(delta_t);

            RelativeTime.push_back(delta_t);
            fabs_RelativeTime.push_back(fabs_delta_t);
        }

        vector<double>::iterator biggest =
                max_element(begin(fabs_RelativeTime), end(fabs_RelativeTime));

        int _ID, ID_RefPoint;

        _ID = distance(begin(fabs_RelativeTime), biggest);

        if (RelativeTime.at(_ID) <= 0.0) {
            ID_RefPoint = _ID - 1;

            if(ID_RefPoint == -1) {
                ID_RefPoint = 0;
            }
        } else {
            ID_RefPoint = _ID;

            if (ID_RefPoint == (SizeOfRefTraj - 1)) {
                ID_RefPoint = SizeOfRefTraj - 2;
            }
        }

        double dis1, dis2, dis_total;

        dis1 = fabs_RelativeTime.at(ID_RefPoint);
        dis2 = fabs_RelativeTime.at(ID_RefPoint + 1);

        dis_total = dis1 + dis2;

        vc_ref =
                ( global_traj_points_.at(ID_RefPoint).v_ref * dis2 +
                global_traj_points_.at(ID_RefPoint + 1).v_ref * dis1 ) /
                dis_total;
        wc_ref =
                ( global_traj_points_.at(ID_RefPoint).w_ref * dis2 +
                global_traj_points_.at(ID_RefPoint + 1).w_ref * dis1 ) /
                dis_total;

        time_predict += predict_step_;

        ref_point_command_.at(i * nu_) = vc_ref;
        ref_point_command_.at(i * nu_ + 1) = wc_ref;
    }
}
