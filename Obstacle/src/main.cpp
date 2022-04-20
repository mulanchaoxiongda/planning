#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include <string>
#include <sys/time.h>

#include "SaveData.h"
#include "RobotModel.h"
#include "TrackingMPC.h"
#include "PlanningMPC.h"
#include "PlanningLattice.h"
#include "TrajectoryCreator.h"
#include "bypass_obstacle_planner.h"

using namespace std;

int main(int argc, char **argv)
{
    struct timeval t_start, t_end;
    gettimeofday(&t_start,NULL);

    SaveData save_result("../tools/SimulationResult.txt");

    SaveData save_trajectory_reference("../data/TrajectoryPoints.txt");

    TrajectoryCreator TrajCreator(&save_trajectory_reference, &save_result);
    TrajCreator.TrajCreator();

    RobotMotionStatePara motion_state = {0.00,  0.00, 0.0/57.3, // x, y, yaw
                                          0.00,  0.00, 0.0,      // v, w, time
                                          0.00,  0.00};           // ax, ay 

    double simulation_step = 0.01;

    RobotModel robot_model(motion_state, simulation_step, &save_result);

    GoalState goal_state = {2.13, 2.13, 90.0/57.3, 0.0, 0.0/57.3}; // x, y, yaw, v, w
    
    PlanningLattice planning_lattice(&robot_model, &save_result);

    PlanningMPC planning_mpc(&robot_model, &save_result);
/////////////////////////////////////////////////////////////////////////////////////////
//     QpSplineSmoother qp_spline_smoother(&save_result);
    
//     deque<CurvePoint> test; // debug
//     qp_spline_smoother.Txt2Vector(test, "../data/RoutingLine.txt");
//     qp_spline_smoother.SetCurvePoints(test);

//     RobotPose test_pose = {0.0, 0.0, 0.0};
//     qp_spline_smoother.SetRobotPose(test_pose);
     
//     deque<SmoothLinePoint> test_line; // debug
//     qp_spline_smoother.GetSmoothLine(test_line);
    
    QpSplineSmoother qp_spline_smoother(&save_result);
    PiecewiseJerkPathOptimization piecewise_jerk_path_optimization(&save_result);
    
    deque<CurvePoint> curve_points;
    qp_spline_smoother.Txt2Vector(curve_points, "../data/RoutingLine.txt");
    piecewise_jerk_path_optimization.SetCurvePoints(curve_points);

    RobotPose robot_pose = {0.0, 0.0, 0.0};
    piecewise_jerk_path_optimization.SetRobotPose(robot_pose);

    piecewise_jerk_path_optimization.LoadOccupyMap("../data/OccupyMap.txt");

    deque<LocalPathPoint> local_path;
    piecewise_jerk_path_optimization.GetLocalPath(local_path);
//////////////////////////////////////////////////////////////////////////////////
    TrackingMPC tracking_mpc(&robot_model, &save_result);

    ControlCommand control_command = {motion_state.v, motion_state.w, 0.0};

    double time = 0.0, control_period = 0.02, planning_period = 0.1;

    int num_planning = round(planning_period / simulation_step);

    int num_control = round(control_period / simulation_step);

    int loop_counter = 0;

    vector<TrajPoint> local_traj_points;
    vector<TrajPoint> optimal_traj_points;

    double time_simulation = 1.0;

    while (time <= time_simulation) {
        if (loop_counter % num_planning == 0) {
            planning_lattice.CalRefTrajectory(local_traj_points, goal_state);

            time_simulation = planning_lattice.GetTimeSimulation();

            planning_mpc.CalRefTrajectory(
                    optimal_traj_points, local_traj_points);
        }

        if (loop_counter % num_control == 0) {
            control_command = tracking_mpc.CalControlCommand(optimal_traj_points);
        }

        robot_model.UpdateMotionState(control_command);

        time = time + simulation_step;
        loop_counter = loop_counter + 1;
    }

    gettimeofday(&t_end,NULL);

    cout << "[planning] Lattice run time(average): "
         << planning_lattice.GetRunningTimeAverage() * 1000.0
         << "ms." <<endl;

    cout << "[planning] MPC run time(average): "
         << planning_mpc.GetRunningTimeAverage() * 1000.0
         << "ms." <<endl;

    cout << "[tracking] MPC run time(average): "
         << tracking_mpc.GetRunningTimeAverage() * 1000.0
         << "ms." <<endl;

    cout << "Program run time: "
         << (double)(t_end.tv_sec - t_start.tv_sec) * 1000.0 +
            (double)(t_end.tv_usec - t_start.tv_usec) / 1000.0
         << "ms." << endl;

    return 0;
}
