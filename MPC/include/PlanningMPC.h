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

class PlanningMPC: public PlanningAlgorithm
{
 public:
     PlanningMPC(RobotModel *p_robot_model, SaveData *p_savedata, GoalState goal_state);
     ~PlanningMPC() {};

     ControlCommand CalRefTrajectory(vector<TrajPoint> &local_traj_points);

     double running_time_average_;


 private:
     bool start_gate_;
     
     int weak_planning_num_;
     int sensor_info_id_;

     double  weak_planning_duration_;

     VectorXd u_pre_;
     int nx_, nu_;

     int np_, nc_;

     MatrixXd q_, r_;

     VectorXd u_min_, u_max_, du_min_, du_max_;

     VectorXd x_;
     MatrixXd a_, b_;

     double running_time_sum_;

     MatrixXd A_, B_, C_;

     int loop_counter_;

     vector<TrajPoint> ref_traj_points_;

     VectorXd u_optimal_;

     bool gate_start_;

     double predict_step_;
     double call_cycle_;

     vector<double> ref_point_command_;

     void ReadInGoalTraj();

     void FindRefPoint();

     void UpdateSensorInfo();

     void CalControlCoefficient();

     void UpdateErrorModel();

     void UpdateIncrementModel();

     void PredictFunc(MatrixXd &phi, MatrixXd &theta);

     void ObjectiveFunc(
             MatrixXd &h, MatrixXd &e, VectorXd &g, MatrixXd kesi,
             MatrixXd phi, MatrixXd theta);

     void ConstraintCondition(
             MatrixXd &A, VectorXd &lb, VectorXd &ub);

     int  OptimizationSolver(
             VectorXd &optimal_solution, MatrixXd p01, VectorXd q01, MatrixXd Ac,
             VectorXd l01, VectorXd u01, int m01, int n01, c_int max_iteration,
             c_float eps_abs);

     void MatrixToCCS(
             MatrixXd matrix_, vector<c_float> *sm_x, c_int &sm_nnz,
             vector<c_int> *sm_i, vector<c_int> *sm_p);

     template <typename T>
     T *CopyData(const std::vector<T> &vec) {
         // T *data = new T[vec.size()];
         T *data = (T*)c_malloc(vec.size()*sizeof(T));
         memcpy(data, vec.data(), sizeof(T) * vec.size());
         return data;
     }

     void UpdateReferenceTrajectory();

     void SmoothTrajcecory();

     void CalPredictForwardCommand();
};