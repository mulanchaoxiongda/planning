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

using namespace std;

int main(int argc, char **argv)
{
    struct timeval t_start, t_end;
    gettimeofday(&t_start,NULL);

    const char* full_result_path = "../tools/SimulationResult.txt";
    SaveData save_result(full_result_path);

    const char* full_trajectory_reference_path = "../data/TrajectoryPoints.txt";

    SaveData save_trajectory_reference(full_trajectory_reference_path);

    TrajectoryCreator TrajCreator(&save_trajectory_reference, &save_result);
    TrajCreator.TrajCreator();

    /* RobotMotionStatePara motion_state = {0.00, -0.00, -0.0/57.3, // x, y, yaw
                                         0.00,  0.0,   0.0,      // v, w, time
                                         0.00,  0.00};           // ax, ay  */

     /* 边界性能测试 */
     RobotMotionStatePara motion_state = {0.00, -0.00, 0.0/57.3, // x, y, yaw
                                         0.00,  0.0,   0.0,      // v, w, time
                                         0.00,  0.00};           // ax, ay 

    double simulation_step = 0.01;

    RobotModel robot_model(motion_state, simulation_step, &save_result);

    /* GoalState goal_state = {1.00, -0.10, 5.0/57.3, 0.0, 0.0/57.3}; // x, y, yaw, v, w */

    /* 边界性能测试 */
    GoalState goal_state = {2.12, 2.12, 90.0/57.3, 0.0, 0.0/57.3}; // x, y, yaw, v, w
    /* GoalState goal_state = {0.0, 3.0, 90.0/57.3, 0.0, 0.0/57.3}; // x, y, yaw, v, w */
    
    PlanningLattice planning_lattice(&robot_model, &save_result);

    PlanningMPC planning_mpc(&robot_model, &save_result);

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
        /* 边界条件没有摸清出，与相机识别算法品质、停车相对位姿场景、
        停车运动过程品质要求、规划算法调较等相关 */
        /* 目标点变化鲁棒性测试 */
        /* if (time >= 2.0) {
             goal_state = {1.00, -0.15, 0.0/57.3, 0.0, 0.0/57.3};
        } */

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
