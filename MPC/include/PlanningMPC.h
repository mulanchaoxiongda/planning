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
     PlanningMPC(RobotModel *p_robot_model, SaveData *p_savedata,
                 GoalState goal_state);
     ~PlanningMPC() {};

     ControlCommand CalRefTrajectory(
            vector<TrajPoint> &local_traj_points);

     ControlCommand CalRefTrajectory(
            vector<TrajPoint> &optimal_traj_points,
            vector<TrajPoint> &local_traj_points);

     double GetRunningTimeAverage();


 private:
     vector<TrajPoint> global_traj_points_;
     RefPoint global_ref_traj_point_;

     /* smoothed motion state for planner to get smooth trajcetory */
     SensorInfo sensor_info_planner_;

     bool start_gate_;
     
     int weak_planning_num_;
     int planner_sensor_info_id_;

     int index_init_point_strong_planner_;

     double  weak_planning_duration_;

     VectorXd u_pre_;

     int nx_, nu_; // number of state variable and control variable

     int np_, nc_; // predict time domain, control time domain

     MatrixXd matrix_q_, matrix_r_;

     VectorXd u_min_, u_max_, du_min_, du_max_;

     VectorXd x_; // control variable
     MatrixXd matrix_a_, matrix_b_;

     double running_time_sum_;

     MatrixXd matrix_A_, matrix_B_, matrix_C_;

     int loop_counter_;

     vector<TrajPoint> ref_traj_points_;

     VectorXd u_optimal_; // optimal control variable
     VectorXd u_opt_storage_;

     bool gate_start_;

     double predict_step_;
     double call_cycle_;

     double running_time_average_;

     vector<double> ref_point_command_;

     void ReadInGoalTraj();

     void GenerateGlobalTraj();

     void FindRefPoint();

     void UpdatePlannerSensorInfo();

     void CalControlCoefficient();

     void UpdateErrorModel();

     void UpdateIncrementModel();

     void Predict(MatrixXd &phi, MatrixXd &theta);

     void CalObjectiveFunc(
             MatrixXd &matrix_h, MatrixXd &matrix_e, VectorXd &matrix_g,
             MatrixXd matrix_kesi, MatrixXd matrix_phi, MatrixXd matrix_theta);

     void CalConstraintConditions(
             MatrixXd &matrix_A, VectorXd &lb, VectorXd &ub);

     int  OptimizationSolver(
             VectorXd &optimal_solution, MatrixXd matrix_p, VectorXd vector_q,
             MatrixXd matrix_Ac, VectorXd vector_l, VectorXd vector_u,
             c_int max_iteration, c_float eps_abs);

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

     void CalPredictForwardCommand();
};
