#include <iostream>
#include <math.h>
#include <iomanip>

#include "TrajectoryCreator.h"

TrajectoryCreator::TrajectoryCreator(
        SaveData *p_save_trajectory_reference, SaveData *p_save_result)
{
    p_save_trajectory_reference_ = p_save_trajectory_reference;
    p_save_result_ = p_save_result;
}

void TrajectoryCreator::TrajCreator()
{
    TrajPoint trajectory_point = {0.0, 0.0, 0.0/57.3, 0.5*0.15*2.5*2.5, 0.0, 0.0}; // x, y, yaw, v, w, t

    p_save_result_->file << "[reference_trajectory] "
                         << " Time " << trajectory_point.t_ref
                         << " x "    << trajectory_point.x_ref
                         << " y "    << trajectory_point.y_ref
                         << " yaw "  << trajectory_point.yaw_ref
                         << " v "    << trajectory_point.v_ref
                         << " w "    << trajectory_point.w_ref
                         << " t "    << trajectory_point.t_ref << endl;

    p_save_trajectory_reference_->file << trajectory_point.x_ref   << "  "
                                       << trajectory_point.y_ref   << "  "
                                       << trajectory_point.yaw_ref << "  "
                                       << trajectory_point.v_ref   << "  "
                                       << trajectory_point.w_ref   << "  "
                                       << trajectory_point.t_ref   << endl;

    while (trajectory_point.t_ref <= 2.3) {
        double speed_command, yaw_ratio_command, radius, simulation_step = 0.05;

        trajectory_point.t_ref = trajectory_point.t_ref+simulation_step;

        /* speed_command = 0.5*0.15*(trajectory_point.t_ref-2.5)*(trajectory_point.t_ref-2.5);

        radius = 100000000.0;

        yaw_ratio_command = speed_command / radius * 0.0; */

        speed_command = 0.5*0.15*(trajectory_point.t_ref-2.5)*(trajectory_point.t_ref-2.5);

        radius = 5.0;

        yaw_ratio_command = speed_command / radius * 1.0;

        trajectory_point.v_ref = speed_command;
        trajectory_point.w_ref = yaw_ratio_command;
        trajectory_point.x_ref =
                trajectory_point.x_ref + trajectory_point.v_ref *
                cos(trajectory_point.yaw_ref) * simulation_step;
        trajectory_point.y_ref =
                trajectory_point.y_ref + trajectory_point.v_ref *
                sin(trajectory_point.yaw_ref) * simulation_step;
        trajectory_point.yaw_ref =
                trajectory_point.yaw_ref + trajectory_point.w_ref *
                simulation_step;

        p_save_result_->file <<"[reference_trajectory] "
                             << " Time " << trajectory_point.t_ref
                             << " x "    << trajectory_point.x_ref
                             << " y "    << trajectory_point.y_ref
                             << " yaw "  << trajectory_point.yaw_ref
                             << " v "    << trajectory_point.v_ref
                             << " w "    << trajectory_point.w_ref
                             << " t "    << trajectory_point.t_ref << endl;

        p_save_trajectory_reference_->file << trajectory_point.x_ref   << "  "
                                           << trajectory_point.y_ref   << "  "
                                           << trajectory_point.yaw_ref << "  "
                                           << trajectory_point.v_ref   << "  "
                                           << trajectory_point.w_ref   << "  "
                                           << trajectory_point.t_ref   << endl;
    }

    cout << "[INFO] creat trajectory successfully !" << endl;
}
