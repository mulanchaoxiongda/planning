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

#include "PlanningLattice.h"
#include "CustomFunction.h"

using namespace std;

PlanningLattice::PlanningLattice(
        RobotModel *p_robot_model, SaveData *p_savedata, GoalState goal_state):
        PlanningAlgorithm::PlanningAlgorithm(
                p_robot_model, p_savedata, goal_state)
{
    start_gate_ = false;

    weak_planning_num_ = 3;

    weak_planning_duration_ = 0.16;

    call_cycle_ = 0.02;

    running_time_sum_ = 0.0;

    loop_counter_ = 0;

    gate_start_ = true;

    GetSensorInfo();

    //ReadInGoalTraj();
    GenerateGlobalTraj();
}

ControlCommand PlanningLattice::CalRefTrajectory(
        vector<TrajPoint> &local_traj_points)
{
    struct timeval t_start, t_end;
    gettimeofday(&t_start,NULL);

    if (start_gate_ == false) {
        GetSensorInfo();
    } else {
        UpdatePlannerSensorInfo();
    }

    gettimeofday(&t_end, NULL);

    loop_counter_++;

    running_time_sum_ = running_time_sum_ + (t_end.tv_sec - t_start.tv_sec) +
                        (double)(t_end.tv_usec - t_start.tv_usec) / 1000000.0;

    running_time_average_ = running_time_sum_ / (double)loop_counter_;

    ControlCommand result = {0.0, 0.0, 0.0};

    return result;
}

void PlanningLattice::ReadInGoalTraj()
{
    string string_temp;

    ifstream read_file;
    read_file.open("../data/TrajectoryPoints.txt", ios::in);

    if (read_file.fail()) {
        cout << "[error] failed to open TrajectoryPoints.txt" << endl;
    } else {
        while (getline(read_file, string_temp)) {
            istringstream is(string_temp);

            double data;

            vector<double> temp_container;
            TrajPoint temp;

            while (!is.eof()) {
                is>>data;
                temp_container.push_back(data);
            }

            temp.x_ref   = temp_container.at(0);
            temp.y_ref   = temp_container.at(1);
            temp.yaw_ref = temp_container.at(2);
            temp.v_ref   = temp_container.at(3);
            temp.w_ref   = temp_container.at(4);
            temp.t_ref   = temp_container.at(5);

            local_traj_points_.push_back(temp);

            temp_container.clear();
            string_temp.clear();
        }
    }

    read_file.close();
    cout << "[INFO] read in reference global route points successfully !"
         << endl;
}

void PlanningLattice::GenerateGlobalTraj()
{
    double distance_agv2goal =
            pow(pow(goal_state_.x - sensor_info_planner_.x, 2.0) + 
            pow(goal_state_.y - sensor_info_planner_.y, 2.0), 0.5);

    double min_relative_dis = 1.0;

    if (distance_agv2goal < min_relative_dis) {
        cout << "[error] agv is too near to goal point!" << endl;
    }

    TrajPoint temp;

    temp.x_ref   = sensor_info_planner_.x;
    temp.y_ref   = sensor_info_planner_.y;
    temp.yaw_ref = sensor_info_planner_.yaw;
    temp.v_ref   = sensor_info_planner_.v;
    temp.w_ref   = sensor_info_planner_.w;
    temp.t_ref   = sensor_info_planner_.t;

    local_traj_points_.push_back(temp);

    double polynomial_step = 0.05;

    double min_speed = 0.01;

    double acceleration = 0.0;

    while (fabs(temp.v_ref) < min_speed) {
        acceleration = 0.1;

        temp.v_ref   = temp.v_ref + acceleration * polynomial_step;
        temp.w_ref   = temp.w_ref;

        temp.yaw_ref = temp.yaw_ref + temp.w_ref * polynomial_step;
        
        temp.x_ref   = temp.x_ref + temp.v_ref * cos(temp.yaw_ref) * polynomial_step;
        temp.y_ref   = temp.y_ref + temp.v_ref * sin(temp.yaw_ref) * polynomial_step;
        
        temp.t_ref   = temp.t_ref + polynomial_step;

        local_traj_points_.push_back(temp);
    }
    
    /* Todo 对时间time_margin、速度speed_except采样，形成备选轨迹集合
    Todo 明确场景与边界条件，建模仿真
    Todo 调参，分析仿真数据，确定代价函数和硬件约束
    Todo 调试，定版
    Todo 五次多项式拼接出曲率不连续 */
    double speed_except = 0.15; // Lattice

    double time_to_goal = 2.5;
    double safe_distance = speed_except * time_to_goal;

    double time_margin = 1.0; // Lattice

    double t0, t1;
    
    t0 = temp.t_ref + 0.0;
    t1 = 
            t0 + (distance_agv2goal - safe_distance) /
            speed_except + time_margin;

    MatrixXd X(6, 1), Y(6, 1);
    
    X << temp.x_ref,
         temp.v_ref * cos(temp.yaw_ref),
         acceleration * cos(temp.yaw_ref),
         goal_state_.x - safe_distance * cos(goal_state_.yaw),
         speed_except * cos(goal_state_.yaw),
         0.0;    

    Y << temp.y_ref,
         temp.v_ref * sin(temp.yaw_ref),
         acceleration * sin(temp.yaw_ref),
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

    int cycle_num = (int)((t1 - t0) / polynomial_step) + 1;

    for (int i = 1; i < cycle_num; i++) {
        double t = t0 + i * polynomial_step;

        MatrixXd matrix_T(3, 6);

        matrix_T << pow(t, 5.0),        pow(t, 4.0),        pow(t, 3.0),       pow(t, 2.0), t,   1.0,
                    5.0 * pow(t, 4.0),  4.0 * pow(t, 3.0),  3.0 * pow(t, 2.0), 2.0 * t,     1.0, 0.0,
                    20.0 * pow(t, 3.0), 12.0 * pow(t, 2.0), 6.0 * t,           2.0,         0.0, 0.0;

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

        double v, yaw, curvature, w;

        v = pow(pow(vx, 2.0) + pow(vy, 2.0), 0.5);

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

        local_traj_points_.push_back(temp);
    }

    if (temp.t_ref < t1) {
        temp.x_ref   = goal_state_.x - safe_distance * cos(goal_state_.yaw);
        temp.y_ref   = goal_state_.y - safe_distance * sin(goal_state_.yaw);
        temp.yaw_ref = goal_state_.yaw;
        temp.v_ref   = speed_except;
        temp.w_ref   = 0.0;
        temp.t_ref   = t1;

        local_traj_points_.push_back(temp);
    }

    temp.x_ref   = goal_state_.x;
    temp.y_ref   = goal_state_.y;
    temp.yaw_ref = goal_state_.yaw;
    temp.v_ref   = goal_state_.v + speed_except;
    temp.w_ref   = goal_state_.w;
    temp.t_ref   = temp.t_ref + safe_distance / speed_except;

    local_traj_points_.push_back(temp);

    cout << "[INFO] generate reference global route points successfully !"
         << endl;
}

void PlanningLattice::UpdatePlannerSensorInfo()
{
    int size_ref_traj = ref_traj_points_.size();

    if (size_ref_traj <= 1 ) {
        cout << "[error] global reference trajectory has only "
             << size_ref_traj << " points " << endl << endl;
    }

    vector<double> relative_time;
    vector<double> fabs_relative_time;

    for (int i = 0; i < size_ref_traj; i++) {
        double delta_t, fabs_delta_t;
                                                                        
        delta_t = 
                p_robot_model_->motion_state_.t - ref_traj_points_.at(i).t_ref;
        
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

    /* weak planning time = weak_planning_num_ * predict_step_ */
    int index_transition_point = ref_point_index + weak_planning_num_;
    
    sensor_info_planner_.t   =
            ref_traj_points_.at(index_transition_point).t_ref;
    sensor_info_planner_.x   =
            ref_traj_points_.at(index_transition_point).x_ref;
    sensor_info_planner_.y   =
            ref_traj_points_.at(index_transition_point).y_ref;
    sensor_info_planner_.yaw =
            ref_traj_points_.at(index_transition_point).yaw_ref;
    sensor_info_planner_.v   =
            ref_traj_points_.at(index_transition_point).v_ref;
    sensor_info_planner_.w   =
            ref_traj_points_.at(index_transition_point).w_ref;

    planner_sensor_info_id_ = ref_point_index;
    index_init_point_strong_planner_ = index_transition_point - 1;
}

double PlanningLattice::GetRunningTimeAverage()
{
    return running_time_average_;
}

void PlanningLattice::SprinkleFunc(
        vector<double> &sample_time, int &num_time,
        vector<double> &sample_speed, int &num_speed,
        vector<double> &sample_distance, int &num_distance)
{
    /* Todo 移到上层函数 */
    double distance_agv2goal =
            pow(pow(goal_state_.x - sensor_info_planner_.x, 2.0) + 
            pow(goal_state_.y - sensor_info_planner_.y, 2.0), 0.5);

    double min_relative_dis = 0.3;

    if (distance_agv2goal < min_relative_dis) {
        cout << "[INFO] agv is near to goal point!" << endl;

        return;
    }

    num_speed = 3;
    sample_speed.resize(num_speed);
    
    sample_speed.at(0) = 0.15;
    sample_speed.at(1) = 0.20;
    sample_speed.at(2) = 0.25;

    double time_to_goal = 2.5;

    num_distance = num_speed;
    sample_distance.resize(num_distance);
    
    sample_distance.at(0) = sample_speed.at(0) * time_to_goal;
    sample_distance.at(1) = sample_speed.at(1) * time_to_goal;
    sample_distance.at(2) = sample_speed.at(2) * time_to_goal;

    double time_interval = 5.0;
    double delta_time = 0.5;
    
    num_time = int(time_interval / delta_time) + 1;

    sample_time.resize(num_time * num_speed);

    double safe_distance, time_benchmark;

    for (int i = 0; i < num_speed; i++) {
        safe_distance = sample_speed.at(i) * time_to_goal;

        time_benchmark =
                (distance_agv2goal - safe_distance) / sample_speed.at(i);

        for (int j = 0; i < num_time; j++) {
            sample_time.at(i * num_speed + j) =
                    time_benchmark + ((double)j - time_interval / delta_time) *                    
                    delta_time;
        }
    }
}

void PlanningLattice::CalPolynomialCurve(
        double time, double speed, double distance)
{
    TrajPoint temp;

    temp.x_ref   = sensor_info_planner_.x;
    temp.y_ref   = sensor_info_planner_.y;
    temp.yaw_ref = sensor_info_planner_.yaw;
    temp.v_ref   = sensor_info_planner_.v;
    temp.w_ref   = sensor_info_planner_.w;
    temp.t_ref   = sensor_info_planner_.t;

    local_traj_points_.push_back(temp);

    double polynomial_step = 0.05;

    double min_speed = 0.01;

    double acceleration = 0.0;

    while (fabs(temp.v_ref) < min_speed) {
        acceleration = 0.1;

        temp.v_ref   = temp.v_ref + acceleration * polynomial_step;
        temp.w_ref   = temp.w_ref;

        temp.yaw_ref = temp.yaw_ref + temp.w_ref * polynomial_step;
        
        temp.x_ref   = temp.x_ref + temp.v_ref * cos(temp.yaw_ref) * polynomial_step;
        temp.y_ref   = temp.y_ref + temp.v_ref * sin(temp.yaw_ref) * polynomial_step;
        
        temp.t_ref   = temp.t_ref + polynomial_step;

        local_traj_points_.push_back(temp);
    }
    
    /* Todo 对时间time_margin、速度speed_except采样，形成备选轨迹集合
    Todo 明确场景与边界条件，建模仿真
    Todo 调参，分析仿真数据，确定代价函数和硬件约束
    Todo 调试，定版
    Todo 五次多项式拼接出曲率不连续 */
    double speed_except  = speed;
    double safe_distance = distance;
    double time_duration = time;

    double t0, t1;
    t0 = temp.t_ref;
    t1 = t0 + time_duration;
            
    MatrixXd X(6, 1), Y(6, 1);
    
    X << temp.x_ref,
         temp.v_ref * cos(temp.yaw_ref),
         acceleration * cos(temp.yaw_ref),
         goal_state_.x - safe_distance * cos(goal_state_.yaw),
         speed_except * cos(goal_state_.yaw),
         0.0;    

    Y << temp.y_ref,
         temp.v_ref * sin(temp.yaw_ref),
         acceleration * sin(temp.yaw_ref),
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

    int cycle_num = (int)((t1 - t0) / polynomial_step) + 1;

    for (int i = 1; i < cycle_num; i++) {
        double t = t0 + i * polynomial_step;

        MatrixXd matrix_T(3, 6);

        matrix_T << pow(t, 5.0),        pow(t, 4.0),        pow(t, 3.0),       pow(t, 2.0), t,   1.0,
                    5.0 * pow(t, 4.0),  4.0 * pow(t, 3.0),  3.0 * pow(t, 2.0), 2.0 * t,     1.0, 0.0,
                    20.0 * pow(t, 3.0), 12.0 * pow(t, 2.0), 6.0 * t,           2.0,         0.0, 0.0;

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

        double v, yaw, curvature, w;

        v = pow(pow(vx, 2.0) + pow(vy, 2.0), 0.5);

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

        local_traj_points_.push_back(temp);
    }

    if (temp.t_ref < t1) {
        temp.x_ref   = goal_state_.x - safe_distance * cos(goal_state_.yaw);
        temp.y_ref   = goal_state_.y - safe_distance * sin(goal_state_.yaw);
        temp.yaw_ref = goal_state_.yaw;
        temp.v_ref   = speed_except;
        temp.w_ref   = 0.0;
        temp.t_ref   = t1;

        local_traj_points_.push_back(temp);
    }

    temp.x_ref   = goal_state_.x;
    temp.y_ref   = goal_state_.y;
    temp.yaw_ref = goal_state_.yaw;
    temp.v_ref   = goal_state_.v + speed_except;
    temp.w_ref   = goal_state_.w;
    temp.t_ref   = temp.t_ref + safe_distance / speed_except;

    local_traj_points_.push_back(temp);

    cout << "[INFO] generate reference local trajectory points successfully !"
         << endl;;
}

void PlanningLattice::ScoringFunc()
{
    double score;

    double score_vmax, score_wmax, score_time, score_jer, score_acc, score_w;
    double weight_vmax, weight_wmax, weight_time, weight_jer, weight_acc, weight_w;

    weight_vmax = 0.0;
    weight_wmax = 0.0;
    weight_time = 0.0;
    weight_jer = 0.0;
    weight_acc = 0.0;
    weight_w = 0.0;

    score_vmax = 0.0;
    score_wmax = 0.0;
    score_time = 0.0;
    score_jer = 0.0;
    score_acc = 0.0;
    score_w = 0.0;

    score = weight_vmax * score_vmax + weight_wmax * score_wmax +
            weight_time * score_time + weight_jer * score_jer +
            weight_acc * score_acc + weight_w * score_w;

    ScoreData temp;
    temp.index = 0; // i * num_speed + j
    temp.score = score;
    temp.expect_speed = 0.0; //
    temp.safe_distance = 0.0; //
    temp.time_duration = 0.0; //
    temp.v_max = 0.0; //
    temp.w_max = 0.0; //

    score_data_.push_back(temp);
}
