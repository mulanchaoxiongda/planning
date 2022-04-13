#pragma once

#include <vector>
#include <eigen3/Eigen/Eigen>
#include <osqp/osqp.h>

#include "RobotModel.h"
#include "InformationFormat.h"
#include "SaveData.h"
#include "PlanningAlgorithm.h"

using namespace std;
using namespace Eigen;

class RobotModel;
class SaveData;

class PlanningLattice: public PlanningAlgorithm
{
 public:
     PlanningLattice(
             RobotModel *p_robot_model, SaveData *p_savedata);

     ~PlanningLattice() {};

     ControlCommand CalRefTrajectory(
             vector<TrajPoint> &local_traj_points);

     ControlCommand CalRefTrajectory(
             vector<TrajPoint> &local_traj_points, GoalState goal_state);

     double GetRunningTimeAverage();

     double GetTimeSimulation();


 private:
     double start_time_polynomial_;

     double opt_speed_;
     double opt_time_;
     double opt_distance_;

     double min_relative_dis_;

     // 避免AGV和托盘碰撞的最小起始停车相对距离，小于此距离规划轨迹需为直线
     double min_init_parking_dis_;
     
     double distance_agv2goal_;

     double step_polynomial_curve_;

     vector<ScoreData> score_data_;

     double time_simulation_;
     
     /* smoothed motion state for planner to get smooth trajcetory */
     SensorInfo sensor_info_planner_;

     bool start_gate_;
     
     int weak_planning_num_;
     int planner_sensor_info_id_;

     int index_init_point_strong_planner_;

     double running_time_sum_;

     int loop_counter_;

     bool gate_start_;

     double predict_step_;
     double call_cycle_;

     double running_time_average_;

     void ReadInGoalTraj();

     void CollisionDetection();

     void UpdatePlannerSensorInfo();

     void SprinkleFunc(
             vector<double> &sample_time, int &num_time,
             vector<double> &sample_speed, int &num_speed,
             vector<double> &sample_distance, int &num_distance);

     void CalPolynomialCurve(
             double time, double speed, double distance, double step);

     void ScoringFunc(
             double except_speed, int speed_index, int time_index,
             int num_time, int &num_alter_traj);
       
     double SelectTrajFunc(int num_time, int &opt_traj_index);

     void CalPolynomialCurve(
             double time, double speed, double distance,
             double step, TrajPoint traj_point);

     double TrajDetection(double except_speed);
};
