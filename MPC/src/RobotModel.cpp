#include <RobotModel.h>
#include <math.h>
#include <iostream>
#include <iomanip>

RobotModel::RobotModel(RobotMotionStatePara motion_state,
                       double step, SaveData *p_savedata)
{
    motion_state_ = motion_state;
    simulation_step_ = step;
    p_savedata_ = p_savedata;
}

RobotMotionStatePara RobotModel::UpdateMotionState(ControlCommand control_command)
{
    bool model_type = 0;

    if (model_type == 0) {
        motion_state_.x =
                motion_state_.x + motion_state_.v *
                cos(motion_state_.yaw) * simulation_step_;
        motion_state_.y =
                motion_state_.y + motion_state_.v *
                sin(motion_state_.yaw) * simulation_step_;
        motion_state_.yaw = motion_state_.yaw + motion_state_.w * simulation_step_;
        motion_state_.v = control_command.speed_command;
        motion_state_.w = control_command.yaw_rate_command;
        motion_state_.t = motion_state_.t + simulation_step_;
    } else {
        double Tv = 0.07;
        double Tw = 0.07;

        double acceleration = (control_command.speed_command - motion_state_.v) / Tv;

        double yaw_acceleration =
                (control_command.yaw_rate_command - motion_state_.w) / Tw;

        motion_state_.v = motion_state_.v + acceleration*simulation_step_;
        motion_state_.w = motion_state_.w + yaw_acceleration*simulation_step_;

        motion_state_.yaw = motion_state_.yaw + motion_state_.w * simulation_step_;

        motion_state_.x =
                motion_state_.x + motion_state_.v *
                cos(motion_state_.yaw) * simulation_step_;
        motion_state_.y =
                motion_state_.y + motion_state_.v *
                sin(motion_state_.yaw) * simulation_step_;
        motion_state_.t = motion_state_.t + simulation_step_;
    }

    p_savedata_->file << "[state_of_robot] "
                      << " Time " << motion_state_.t
                      << " x "    << motion_state_.x
                      << " y "    << motion_state_.y
                      << " yaw "  << motion_state_.yaw
                      << " v "    << motion_state_.v
                      << " w "    << motion_state_.w
                      << " t "    << motion_state_.t   << endl;

    return motion_state_;
}