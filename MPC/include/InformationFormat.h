#pragma once

struct RobotMotionStatePara
{
    double x;
    double y;
    double yaw;
    double v;
    double w;
    double t;
};

struct TrajPoint
{
    double x_ref;
    double y_ref;
    double yaw_ref;
    double v_ref;
    double w_ref;
    double t_ref;
};

struct RefPoint
{
    double x;
    double y;
    double yaw;
    double v;
    double w;
    double t;
};

struct SensorInfo
{
    double x;
    double y;
    double yaw;
    double v;
    double w;
    double t;
};

struct TrackingErr
{
    double err_sx;
    double err_sy;
    double err_yaw;
    double t;
};

struct ControlCommand
{
    double speed_command;
    double yaw_rate_command;
    double t;
};

struct GoalState
{
    double x;
    double y;
    double yaw;
    double v;
    double w;
};
