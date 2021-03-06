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
        RobotModel *p_robot_model, SaveData *p_savedata):
        PlanningAlgorithm::PlanningAlgorithm(p_robot_model, p_savedata)
{
    start_gate_ = true;

    min_relative_dis_ = 0.5;

    min_init_parking_dis_ = 0.55;

    weak_planning_num_ = 0;

    call_cycle_ = 0.02;

    running_time_sum_ = 0.0;

    loop_counter_ = 0;

    gate_start_ = true;
}

ControlCommand PlanningLattice::CalRefTrajectory(
        vector<TrajPoint> &local_traj_points)
{
    ControlCommand result = {0.0, 0.0, 0.0};

    return result;
}

ControlCommand PlanningLattice::CalRefTrajectory(
        vector<TrajPoint> &local_traj_points, GoalState goal_state)
{
    struct timeval t_start, t_end;
    gettimeofday(&t_start,NULL);

    ControlCommand result = {0.0, 0.0, 0.0};

    goal_state_ = goal_state;
    
    if (start_gate_ == true) {
        start_gate_ = false;

        GetSensorInfo();

        memcpy(&sensor_info_planner_, &sensor_info_, sizeof(SensorInfo));

        distance_agv2goal_ =
                pow(pow(goal_state_.x - sensor_info_planner_.x, 2.0) + 
                pow(goal_state_.y - sensor_info_planner_.y, 2.0), 0.5);

        if (distance_agv2goal_ < min_relative_dis_) {
            cout << "[info] agv is too near to goal point!" << endl;

            return result;
        }

        vector<double> sample_time, sample_speed, sample_distance;
        int num_time, num_speed, num_distance;

        SprinkleFunc(
                sample_time, num_time,
                sample_speed, num_speed,
                sample_distance, num_distance);

        step_polynomial_curve_ = 0.2;

        int num_alter_traj = 0;

        for (int i = 0; i < num_speed; i++) {
            for (int j = 0; j < num_time; j++) {
                CalPolynomialCurve(
                        sample_time.at(i * num_time + j), sample_speed.at(i),
                        sample_distance.at(i), step_polynomial_curve_);

                ScoringFunc(sample_speed.at(i), i, j, num_time, num_alter_traj);
            }
        }

        if (num_alter_traj == 0) {
            cout << "[error] there is no reference trajectory founded!" << endl;
            
            return result;
        }

        int opt_traj_index;

        double score = SelectTrajFunc(num_time, opt_traj_index);

        /* ?????????????????????????????????????????????
        ???????????????AGV??????????????????????????????????????????????????????
            ???AGV???????????????????????????????????????????????????????????????
             ????????????????????????????????????????????????????????????????????????AGV?????????????????????
        ??????????????????????????????
        ???????????????????????????????????????????????? */
        if (score > 100000.0) {
            cout << "[error] there have no reference trajectory founded!"
                 << endl;

            // ????????????????????????????????????????????????????????????(???????????????) // Todo: ??????
            // ????????????:??????????????????????????????????????????????????????????????????????????????????????????
            
            return result;
        }

        int speed_index = int(opt_traj_index / num_time);

        opt_speed_ = sample_speed.at(speed_index);
        opt_time_ = sample_time.at(opt_traj_index);
        opt_distance_ = sample_distance.at(speed_index);

        double dis_margin = 0.2;
        min_relative_dis_ = opt_distance_ + dis_margin;

        step_polynomial_curve_ = 0.05;

        CalPolynomialCurve(
                opt_time_, opt_speed_, opt_distance_, step_polynomial_curve_);

    } else {
        GetSensorInfo();

        /* ??????????????????????????????????????????????????????????????????????????????????????????AGV???????????????
        ?????????????????????????????????????????????AGV??????????????????????????????????????????AGV??????????????????
        ??????????????????????????????????????????????????????????????????????????????????????????????????????AGV?????????
        ?????????????????????????????????????????????AGV??????????????????????????????????????????????????????????????????
        ?????????????????????????????????????????????????????????????????????AGV??????????????????????????????????????????
        ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        ???????????????????????????????????????????????????????????????????????????????????????????????????????????? */
        
        /* ????????????:??????s-l + s-t???????????????????????????AGV??????????????????????????????????????????
        s-l????????????????????????????????????????????????s-t?????????????????????????????????????????????????????????
        ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        ????????????????????????????????? */
        
        /* ?????????????????????:??????????????????????????????????????????????????????????????????AGV??????????????????
        ???AGV?????????????????????????????????AGV??????????????????????????????s-t????????????????????????????????????
        ????????? */
        
        UpdatePlannerSensorInfo();

        distance_agv2goal_ =
                pow(pow(goal_state_.x - sensor_info_planner_.x, 2.0) + 
                pow(goal_state_.y - sensor_info_planner_.y, 2.0), 0.5);

        if (distance_agv2goal_ < min_relative_dis_) {
            /* cout << "[info] agv is too near to goal point!  "
                 << " t: " << sensor_info_planner_.t
                 << " x: " << sensor_info_planner_.x
                 << " y: " << sensor_info_planner_.y << endl; */

            return result;
        }

        vector<TrajPoint> temp;

        for (int i = planner_sensor_info_id_; 
             i < index_init_point_strong_planner_; i++) {
            temp.push_back(local_traj_points_.at(i));
        }

        TrajPoint init_point_strong_planning;
        memcpy(
                &init_point_strong_planning,
                &local_traj_points_.at(index_init_point_strong_planner_),
                sizeof(TrajPoint));

        local_traj_points_.clear();

        local_traj_points_.assign(temp.begin(), temp.end());

        double opt_time;

        if (sensor_info_planner_.t < start_time_polynomial_) {
            opt_time = opt_time_;
        } else {
            opt_time =
                    start_time_polynomial_ + opt_time_ - sensor_info_planner_.t;
        }

        CalPolynomialCurve(
                opt_time, opt_speed_, opt_distance_, 
                step_polynomial_curve_, init_point_strong_planning);

        double score;
        score = TrajDetection(opt_speed_);

        /* ?????????????????????????????????????????????
        ???????????????AGV???????????????????????????????????????????????????
        ??????????????????????????????
        ????????????????????????????????????opt_speed/time/distance???????????????????????????????????? */
        if (score > 100000.0) {
            cout << "[error] there have no reference trajectory founded!"
                 << endl;

            // ???????????????????????????????????????????????????????????? // Todo: ??????
            // ????????????:???????????????????????????????????????????????????????????????????????????????????????
            
            return result;
        }
    }

    local_traj_points.assign(
                local_traj_points_.begin(), local_traj_points_.end());

    gettimeofday(&t_end, NULL);

    loop_counter_++;

    running_time_sum_ = running_time_sum_ + (t_end.tv_sec - t_start.tv_sec) +
                        (double)(t_end.tv_usec - t_start.tv_usec) / 1000000.0;

    static bool gate_first_planning = 1;
    if (gate_first_planning == 1) {
        gate_first_planning = 0;
        cout << "[planning] Lattice run time(first planning): "
             << running_time_sum_ * 1000.0 << "ms." << endl;
        
        running_time_sum_ = 0.0;
    }

    running_time_average_ = running_time_sum_ / (double)loop_counter_;

    p_savedata_->file << "[goal_state_planning] "
                      << "Time "         << sensor_info_.t
                      << "x_goal "       << goal_state_.x
                      << "y_goal "       << goal_state_.y
                      << "yaw_goal "     << goal_state_.yaw
                      << "v_goal "       << goal_state_.v
                      << "w_goal "       << goal_state_.w
                      << "Time "         << sensor_info_planner_.t
                      << "loop_counter " << (double)loop_counter_ << endl;

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
    cout << "[info] read in reference local trajectory points successfully !"
         << endl;
}

void PlanningLattice::CollisionDetection()
{
    ;
}

void PlanningLattice::UpdatePlannerSensorInfo()
{
    int size_ref_traj = local_traj_points_.size();

    if (size_ref_traj <= 1 ) {
        cout << "[error] global reference trajectory has only "
             << size_ref_traj << " points " << endl << endl;
    }

    vector<double> relative_time;
    vector<double> fabs_relative_time;

    for (int i = 0; i < size_ref_traj; i++) {
        double delta_t, fabs_delta_t;
                                          
        delta_t = 
                sensor_info_.t - local_traj_points_.at(i).t_ref;
        
        fabs_delta_t = -1.0 * fabs(delta_t);

        relative_time.push_back(delta_t);
        fabs_relative_time.push_back(fabs_delta_t);
    }

    vector<double>::iterator biggest =
            max_element(begin(fabs_relative_time), end(fabs_relative_time));

    int ref_point_index = distance(begin(fabs_relative_time), biggest);

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

    /* weak planning time = weak_planning_num_ * step_polynomial_curve_ */
    index_init_point_strong_planner_ = ref_point_index + weak_planning_num_;

    sensor_info_planner_.t   =
            local_traj_points_.at(index_init_point_strong_planner_).t_ref;
    sensor_info_planner_.x   =
            local_traj_points_.at(index_init_point_strong_planner_).x_ref;
    sensor_info_planner_.y   =
            local_traj_points_.at(index_init_point_strong_planner_).y_ref;
    sensor_info_planner_.yaw =
            local_traj_points_.at(index_init_point_strong_planner_).yaw_ref;
    sensor_info_planner_.v   =
            local_traj_points_.at(index_init_point_strong_planner_).v_ref;
    sensor_info_planner_.w   =
            local_traj_points_.at(index_init_point_strong_planner_).w_ref;

    planner_sensor_info_id_ = ref_point_index;
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
    num_speed = 6;
    sample_speed.resize(num_speed);
    
    for (int i = 0; i < num_speed; i++) {
        sample_speed.at(i) = 0.05 * double(i + 1);
    }
    
    // ???????????????AGV?????????????????????????????? + 0.5s
    double time_to_goal = 2.5;

    num_distance = num_speed;
    sample_distance.resize(num_distance);

    for (int i = 0; i < num_distance; i++) {
        sample_distance.at(i) = sample_speed.at(i) * time_to_goal;

        if (sample_distance.at(i) < min_init_parking_dis_) {
            sample_distance.at(i) = min_init_parking_dis_;
        }
    }
    
    double time_interval; // 3.0
    double delta_time; // 1.0

    double time_benchmark;

    for (int i = 0; i < num_speed; i++) {
        time_benchmark =
                (distance_agv2goal_ - sample_distance.at(i)) /
                sample_speed.at(i);

        time_interval = time_benchmark * 1.0; // ????????????

        delta_time = time_benchmark * 0.05; // ????????????

        num_time = round(time_interval / delta_time); // ????????????

        sample_time.resize(num_time * num_speed);

        // ????????????0.1, 0.2, 0.3, 0.4, 0.5??? * time_benchmark + time_benchmark
        for (int j = 0; j < num_time; j++) {
            sample_time.at(i * num_time + j) =
                    time_benchmark + (double)(j + 1) * delta_time;
        }
    }
}

void PlanningLattice::CalPolynomialCurve(
        double time, double speed, double distance, double step)
{
    TrajPoint temp;

    if (time > 0.0) {
        temp.x_ref   = sensor_info_planner_.x;
        temp.y_ref   = sensor_info_planner_.y;
        temp.yaw_ref = sensor_info_planner_.yaw;
        temp.v_ref   = sensor_info_planner_.v;
        temp.w_ref   = sensor_info_planner_.w;
        temp.t_ref   = sensor_info_planner_.t;

        temp.ax_ref = sensor_info_.ax;
        temp.ay_ref = sensor_info_.ay;

        local_traj_points_.push_back(temp);

        double polynomial_step = step;

        double min_speed = 0.01;

        double acceleration = 0.0;

        while (fabs(temp.v_ref) < min_speed) {
            acceleration = 0.1;

            temp.v_ref = temp.v_ref + acceleration * polynomial_step;
            temp.w_ref = temp.w_ref;

            temp.yaw_ref = temp.yaw_ref + temp.w_ref * polynomial_step;
            
            temp.x_ref = 
                    temp.x_ref + temp.v_ref * cos(temp.yaw_ref) *
                    polynomial_step;
            
            temp.y_ref =
                    temp.y_ref + temp.v_ref * sin(temp.yaw_ref) *
                    polynomial_step;
            
            temp.t_ref   = temp.t_ref + polynomial_step;

            temp.ax_ref = acceleration * cos(temp.yaw_ref);
            temp.ay_ref = acceleration * sin(temp.yaw_ref);

            local_traj_points_.push_back(temp);
        }

        double speed_except  = speed;
        double safe_distance = distance;
        double time_duration = time;

        double t0, t1;
        t0 = temp.t_ref;
        t1 = t0 + time_duration;

        start_time_polynomial_ = t0;

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

            temp.ax_ref = ax;
            temp.ay_ref = ay;

            local_traj_points_.push_back(temp);
        }

        if (temp.t_ref < t1) {
            temp.x_ref   =
                    goal_state_.x - safe_distance * cos(goal_state_.yaw);
            temp.y_ref   =
                    goal_state_.y - safe_distance * sin(goal_state_.yaw);
            temp.yaw_ref = goal_state_.yaw;
            temp.v_ref   = speed_except;
            temp.w_ref   = 0.0;
            temp.t_ref   = t1;

            temp.ax_ref = 0.0;
            temp.ay_ref = 0.0;

            local_traj_points_.push_back(temp);
        }

        temp.x_ref   = goal_state_.x;
        temp.y_ref   = goal_state_.y;
        temp.yaw_ref = goal_state_.yaw;
        temp.v_ref   = goal_state_.v + speed_except;
        temp.w_ref   = goal_state_.w;
        temp.t_ref   = temp.t_ref + safe_distance / speed_except;

        temp.ax_ref = 0.0;
        temp.ay_ref = 0.0;

        local_traj_points_.push_back(temp);
    } else {
        temp.x_ref   = sensor_info_planner_.x;
        temp.y_ref   = sensor_info_planner_.y;
        temp.yaw_ref = sensor_info_planner_.yaw;
        temp.v_ref   = sensor_info_planner_.v;
        temp.w_ref   = 50.0/57.3;
        temp.t_ref   = sensor_info_planner_.t;

        temp.ax_ref = 0.0;
        temp.ay_ref = 0.0;

        for (int i = 0; i < 5; i++) {
            local_traj_points_.push_back(temp);
        }
    }

    time_simulation_ = temp.t_ref;
}

void PlanningLattice::CalPolynomialCurve(
        double time, double speed, double distance,
        double step, TrajPoint traj_point)
{
    TrajPoint temp;

    if (time > 0.0) {
        temp.x_ref   = traj_point.x_ref;
        temp.y_ref   = traj_point.y_ref;
        temp.yaw_ref = traj_point.yaw_ref;
        temp.v_ref   = traj_point.v_ref;
        temp.w_ref   = traj_point.w_ref;
        temp.t_ref   = traj_point.t_ref;

        temp.ax_ref = traj_point.ax_ref;
        temp.ay_ref = traj_point.ay_ref;

        local_traj_points_.push_back(temp);

        double polynomial_step = step;

        double min_speed = 0.01;

        double acceleration = 0.0;

        while (fabs(temp.v_ref) < min_speed) {
            acceleration = 0.1;

            temp.v_ref = temp.v_ref + acceleration * polynomial_step;
            temp.w_ref = temp.w_ref;

            temp.yaw_ref = temp.yaw_ref + temp.w_ref * polynomial_step;
            
            temp.x_ref = 
                    temp.x_ref + temp.v_ref * cos(temp.yaw_ref) *
                    polynomial_step;
            
            temp.y_ref =
                    temp.y_ref + temp.v_ref * sin(temp.yaw_ref) *
                    polynomial_step;
            
            temp.t_ref   = temp.t_ref + polynomial_step;

            temp.ax_ref = acceleration * cos(temp.yaw_ref);
            temp.ay_ref = acceleration * sin(temp.yaw_ref);

            local_traj_points_.push_back(temp);
        }

        double speed_except  = speed;
        double safe_distance = distance;
        double time_duration = time;

        double t0, t1;
        t0 = temp.t_ref;
        t1 = t0 + time_duration;

        MatrixXd X(6, 1), Y(6, 1);
        
        X << temp.x_ref,
             temp.v_ref * cos(temp.yaw_ref),
             temp.ax_ref,
             goal_state_.x - safe_distance * cos(goal_state_.yaw),
             speed_except * cos(goal_state_.yaw),
             0.0;

        Y << temp.y_ref,
             temp.v_ref * sin(temp.yaw_ref),
             temp.ay_ref,
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

            temp.ax_ref = ax;
            temp.ay_ref = ay;

            local_traj_points_.push_back(temp);
        }

        if (temp.t_ref < t1) {
            temp.x_ref   =
                    goal_state_.x - safe_distance * cos(goal_state_.yaw);
            temp.y_ref   =
                    goal_state_.y - safe_distance * sin(goal_state_.yaw);
            temp.yaw_ref = goal_state_.yaw;
            temp.v_ref   = speed_except;
            temp.w_ref   = 0.0;
            temp.t_ref   = t1;

            temp.ax_ref = 0.0;
            temp.ay_ref = 0.0;

            local_traj_points_.push_back(temp);
        }

        double parking_distance = 0.1;

        temp.x_ref   = goal_state_.x - parking_distance * cos(goal_state_.yaw);
        temp.y_ref   = goal_state_.y - parking_distance * sin(goal_state_.yaw);
        temp.yaw_ref = goal_state_.yaw;
        temp.v_ref   = goal_state_.v + speed_except;
        temp.w_ref   = goal_state_.w;
        temp.t_ref   =
                temp.t_ref + (safe_distance - parking_distance) / speed_except;

        temp.ax_ref = 0.0;
        temp.ay_ref = 0.0;

        local_traj_points_.push_back(temp);

        temp.x_ref   = goal_state_.x;
        temp.y_ref   = goal_state_.y;
        temp.yaw_ref = goal_state_.yaw;
        temp.v_ref   = goal_state_.v + speed_except;
        temp.w_ref   = goal_state_.w;
        temp.t_ref   = temp.t_ref + parking_distance / speed_except;

        temp.ax_ref = 0.0;
        temp.ay_ref = 0.0;

        local_traj_points_.push_back(temp);

        double parking_step = 0.000001;

        // ???????????????????????????????????????????????????????????????????????????
        // ????????????????????????????????????vector??????
        int traj_point_margin = 10;
        for (int i = 0; i < traj_point_margin; i++) {
            temp.x_ref   =
                    goal_state_.x + i * parking_step *
                    speed_except * cos(goal_state_.yaw);
            temp.y_ref   =
                    goal_state_.y + i * parking_step *
                    speed_except * sin(goal_state_.yaw);
            temp.yaw_ref = goal_state_.yaw;
            temp.v_ref   = goal_state_.v + speed_except;
            temp.w_ref   = goal_state_.w;
            temp.t_ref   = temp.t_ref + i * parking_step;

            temp.ax_ref = 0.0;
            temp.ay_ref = 0.0;

            local_traj_points_.push_back(temp);
        }
    } else {
        temp.x_ref   = sensor_info_planner_.x;
        temp.y_ref   = sensor_info_planner_.y;
        temp.yaw_ref = sensor_info_planner_.yaw;
        temp.v_ref   = sensor_info_planner_.v;
        temp.w_ref   = 50.0/57.3; // trajectory to be deleted
        temp.t_ref   = sensor_info_planner_.t;

        temp.ax_ref = 0.0;
        temp.ay_ref = 0.0;

        for (int i = 0; i < 5; i++) {
            local_traj_points_.push_back(temp);
        }
    }

    time_simulation_ = temp.t_ref;
}

void PlanningLattice::ScoringFunc(
        double except_speed, int speed_index, int time_index,
        int num_time, int &num_alter_traj)
{
    int size_traj_points = local_traj_points_.size();

    vector<double> t_storage; // time
    vector<double> v_storage; // speed
    vector<double> w_storage; // omega
    vector<double> a_storage; // acc
    vector<double> j_storage; // jerk
    vector<double> pos_x, pos_y; // obs

    for (int i = 0; i < size_traj_points; i++) {
        t_storage.push_back(local_traj_points_.at(i).t_ref);
        
        v_storage.push_back(local_traj_points_.at(i).v_ref);

        w_storage.push_back(fabs(local_traj_points_.at(i).w_ref));

        pos_x.push_back(local_traj_points_.at(i).x_ref);
        pos_y.push_back(local_traj_points_.at(i).y_ref);
    }

    vector<double>::iterator biggest_v =
            max_element(begin(v_storage), end(v_storage));

    int vmax_index = distance(begin(v_storage), biggest_v);
    
    double v_max = v_storage.at(vmax_index);

    vector<double>::iterator biggest_w =
            max_element(begin(w_storage), end(w_storage));

    int wmax_index = distance(begin(w_storage), biggest_w);

    double w_max = w_storage.at(wmax_index);

    for (int i = 0; i < size_traj_points - 1; i++) {
        double acc;

        if (v_storage.at(i) == v_storage.at(i + 1)) {
            acc = 0.0;
        } else {
            acc =
                (v_storage.at(i + 1) - v_storage.at(i)) /
                (t_storage.at(i + 1) - t_storage.at(i));
        }

        a_storage.push_back(-1.0 * acc);
    }
    
    for (int i = 0; i < size_traj_points - 2; i++) {
        double jerk;
        
        if (a_storage.at(i) == a_storage.at(i + 1)) {
            jerk = 0.0;
        } else {
            jerk =
                (a_storage.at(i + 1) - a_storage.at(i)) /
                (t_storage.at(i + 1) - t_storage.at(i));
        }

        j_storage.push_back(jerk);
    }
    
    vector<double>::iterator biggest_a =
            max_element(begin(a_storage), end(a_storage));

    int amax_index = distance(begin(a_storage), biggest_a);

    double a_min = -1.0 * a_storage.at(amax_index);

    double dis_obs_min = 10000.0;

    // ???????????????????????????????????????????????????????????????????????????
    // ???????????????????????????/AGV???????????????????????????????????????????????????????????????
    // Lattice????????????????????????????????????MPC???????????????????????????
    // PS???????????????TrajDetection()?????????????????????????????????????????????
    for (int i = 0; i < size_traj_points - 1; i++) {
        FindNearestObsDis(pos_x.at(i), pos_y.at(i));

        if (dis_obs_min > dis_obs_near_) {
            dis_obs_min = dis_obs_near_;
        }
    }

    double score;

    if (v_max >= except_speed * 1.1 || fabs(w_max) >= 20.0 / 57.3 ||
        a_min <= -0.05 || dis_obs_min <= radius_agv_) {
        score = 100000000.0;
    } else {
        num_alter_traj++;

        double score_vmax, score_time, score_jer, score_acc, score_w;
        double weight_vmax, weight_time, weight_jer, weight_acc, weight_w;

        weight_vmax = 1.00;
        weight_time = 0.01;
        weight_jer = 1.0;
        weight_acc = 0.5;
        weight_w = 0.2;

        score_vmax = v_max / except_speed - 1.0;        
        
        score_time =
                local_traj_points_.at(size_traj_points - 1).t_ref;

        score_jer = 0.0;
        
        int i;

        for (i = 0; i < size_traj_points - 2; i++) {
            double delta_time =
                    local_traj_points_.at(i + 1).t_ref -
                    local_traj_points_.at(i).t_ref;

            score_jer = score_jer + fabs(j_storage.at(i)) * delta_time;
        }

        score_jer =
                score_jer + fabs(j_storage.at(i - 1)) *
                (t_storage.at(i + 1) -
                 t_storage.at(i - 1));

        score_acc = 0.0;
        score_w = 0.0;

        double delta_time;
        
        for (i = 0; i < size_traj_points-1; i++) {
            delta_time =
                    local_traj_points_.at(i + 1).t_ref -
                    local_traj_points_.at(i).t_ref;

            score_w = score_w + w_storage.at(i) * delta_time;

            score_acc = score_acc + fabs(a_storage.at(i)) * delta_time;
        }
        score_w =
                score_w + w_storage.at(i) *
                (t_storage.at(i) - t_storage.at(i - 1));

        score_acc =
                score_acc + fabs(a_storage.at(i - 1)) *
                (t_storage.at(i) - t_storage.at(i - 1));

        score = weight_vmax * score_vmax + weight_time * score_time +
                weight_jer * score_jer + weight_acc * score_acc +
                weight_w * score_w;

        /* cout << "[DEBUG] score:" << score
             << " v:" << weight_vmax * score_vmax
             << " t:" << weight_time * score_time
             << " jer:" << weight_jer * score_jer
             << " acc:" << weight_acc * score_acc
             << " w:" << weight_w * score_w
             << endl; */
    }

    ScoreData temp;
    temp.index = speed_index * num_time + time_index;
    temp.score = score;

    score_data_.push_back(temp);

    local_traj_points_.clear();
}

double PlanningLattice::SelectTrajFunc(int num_time, int &opt_traj_index)
{
    int size_score_data = score_data_.size();

    vector<double> score_storage;
    
    for (int i = 0; i < size_score_data; i++) {
        score_storage.push_back(-1.0 * score_data_.at(i).score);
    }

    vector<double>::iterator biggest_score =
            max_element(begin(score_storage), end(score_storage));

    int score_min_index = distance(begin(score_storage), biggest_score);
    
    opt_traj_index = score_data_.at(score_min_index).index;

    double score = score_data_.at(opt_traj_index).score;

    score_data_.clear();

    return score;
}

double PlanningLattice::GetTimeSimulation()
{
    return time_simulation_;
}

double PlanningLattice::TrajDetection(double except_speed)
{
    int size_traj_points = local_traj_points_.size();

    vector<double> t_storage; // time
    vector<double> v_storage; // speed
    vector<double> w_storage; // omega
    vector<double> a_storage; // acc
    vector<double> j_storage; // jerk
    vector<double> pos_x, pos_y; // obs

    for (int i = 0; i < size_traj_points; i++) {
        t_storage.push_back(local_traj_points_.at(i).t_ref);
        
        v_storage.push_back(local_traj_points_.at(i).v_ref);

        w_storage.push_back(fabs(local_traj_points_.at(i).w_ref));

        pos_x.push_back(local_traj_points_.at(i).x_ref);
        pos_y.push_back(local_traj_points_.at(i).y_ref);
    }
    
    vector<double>::iterator biggest_v =
            max_element(begin(v_storage), end(v_storage));

    int vmax_index = distance(begin(v_storage), biggest_v);
    
    double v_max = v_storage.at(vmax_index);

    vector<double>::iterator biggest_w =
            max_element(begin(w_storage), end(w_storage));

    int wmax_index = distance(begin(w_storage), biggest_w);

    double w_max = w_storage.at(wmax_index);

    for (int i = 0; i < size_traj_points - 1; i++) {
        double acc;

        if (v_storage.at(i) == v_storage.at(i + 1)) {
            acc = 0.0;
        } else {
            acc =
                (v_storage.at(i + 1) - v_storage.at(i)) /
                (t_storage.at(i + 1) - t_storage.at(i));
        }

        a_storage.push_back(-1.0 * acc);
    }
    
    for (int i = 0; i < size_traj_points - 2; i++) {
        double jerk;
        
        if (a_storage.at(i) == a_storage.at(i + 1)) {
            jerk = 0.0;
        } else {
            jerk =
                (a_storage.at(i + 1) - a_storage.at(i)) /
                (t_storage.at(i + 1) - t_storage.at(i));
        }

        j_storage.push_back(jerk);
    }
    
    vector<double>::iterator biggest_a =
            max_element(begin(a_storage), end(a_storage));

    int amax_index = distance(begin(a_storage), biggest_a);

    double a_min = -1.0 * a_storage.at(amax_index);

    double dis_obs_min = 10000.0;

    for (int i = 0; i < size_traj_points - 1; i++) {
        FindNearestObsDis(pos_x.at(i), pos_y.at(i));

        if (dis_obs_min > dis_obs_near_) {
            dis_obs_min = dis_obs_near_;
        }
    }

    double score;

    // ??????????????????????????????????????? // Todo: ??????
    if (v_max >= except_speed * 1.3 || fabs(w_max) >= 30.0 / 57.3 ||
        a_min <= -0.15  || dis_obs_min <= dis2obs_min_) {
        score = 100000000.0;
    } else {
        double score_vmax, score_time, score_jer, score_acc, score_w;
        double weight_vmax, weight_time, weight_jer, weight_acc, weight_w;

        weight_vmax = 1.00;
        weight_time = 0.01;
        weight_jer = 1.0;
        weight_acc = 0.5;
        weight_w = 0.2;

        score_vmax = v_max / except_speed - 1.0;        
        
        score_time =
                local_traj_points_.at(size_traj_points - 1).t_ref;

        score_jer = 0.0;
        
        int i;

        for (i = 0; i < size_traj_points - 2; i++) {
            double delta_time =
                    local_traj_points_.at(i + 1).t_ref -
                    local_traj_points_.at(i).t_ref;

            score_jer = score_jer + fabs(j_storage.at(i)) * delta_time;
        }

        score_jer =
                score_jer + fabs(j_storage.at(i - 1)) *
                (t_storage.at(i + 1) -
                 t_storage.at(i - 1));

        score_acc = 0.0;
        score_w = 0.0;

        double delta_time;
        
        for (i = 0; i < size_traj_points-1; i++) {
            delta_time =
                    local_traj_points_.at(i + 1).t_ref -
                    local_traj_points_.at(i).t_ref;

            score_w = score_w + w_storage.at(i) * delta_time;

            score_acc = score_acc + fabs(a_storage.at(i)) * delta_time;
        }
        score_w =
                score_w + w_storage.at(i) *
                (t_storage.at(i) - t_storage.at(i - 1));

        score_acc =
                score_acc + fabs(a_storage.at(i - 1)) *
                (t_storage.at(i) - t_storage.at(i - 1));

        score = weight_vmax * score_vmax + weight_time * score_time +
                weight_jer * score_jer + weight_acc * score_acc +
                weight_w * score_w;
    }

    return score;
}

