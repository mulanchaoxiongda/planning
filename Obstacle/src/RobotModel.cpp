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

    v_pre_ = motion_state_.v;
}

RobotMotionStatePara RobotModel::UpdateMotionState(ControlCommand control_command)
{
    bool model_type = 1;

    if (model_type == 0) {
        motion_state_.w = control_command.yaw_rate_command;
        motion_state_.v = control_command.speed_command;

        motion_state_.yaw = motion_state_.yaw + motion_state_.w * simulation_step_;
        
        motion_state_.x =
                motion_state_.x + motion_state_.v *
                cos(motion_state_.yaw) * simulation_step_;
        motion_state_.y =
                motion_state_.y + motion_state_.v *
                sin(motion_state_.yaw) * simulation_step_;
        
        motion_state_.t = motion_state_.t + simulation_step_;

        double acc_speed, acc_omega;

        acc_speed = (motion_state_.v - v_pre_) / simulation_step_;
        acc_omega = motion_state_.v * motion_state_.w;
        
        motion_state_.ax = acc_speed * cos(motion_state_.yaw) -
                acc_omega * sin(motion_state_.yaw);
                
        motion_state_.ay = acc_speed * sin(motion_state_.yaw) +
                acc_omega * cos(motion_state_.yaw);
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

        double acc_speed, acc_omega;

        acc_speed = acceleration;
        acc_omega = motion_state_.v * motion_state_.w;
        
        motion_state_.ax = acc_speed * cos(motion_state_.yaw) -
                acc_omega * sin(motion_state_.yaw);

        motion_state_.ay = acc_speed * sin(motion_state_.yaw) +
                acc_omega * cos(motion_state_.yaw);
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

RobotMotionStatePara RobotModel::GetRobotMotionState()
{
    return motion_state_;
}