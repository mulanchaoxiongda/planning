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
     PlanningLattice(RobotModel *p_robot_model, SaveData *p_savedata,
                 GoalState goal_state);
     ~PlanningLattice() {};

     ControlCommand CalRefTrajectory(
            vector<TrajPoint> &local_traj_points);

     double GetRunningTimeAverage();

     double GetTimeSimulation();


 private:
     vector<ScoreData> score_data_;

     double time_simulation_;

     double distance_agv2goal_;
     
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

     void UpdatePlannerSensorInfo();

     void SprinkleFunc(
            vector<double> &sample_time, int &num_time,
            vector<double> &sample_speed, int &num_speed,
            vector<double> &sample_distance, int &num_distance);

     void CalPolynomialCurve(double time, double speed, double distance);

     void ScoringFunc(
            double except_speed, int speed_index, int time_index, int num_time);
       
     void SelectTrajFunc(int num_time, int &opt_traj_index);
};
