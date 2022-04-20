#pragma once

#include <deque>
#include <vector>
#include <eigen3/Eigen/Eigen>

#include "InformationFormat.h"
#include "SaveData.h"
#include "curve_smoother.h"

using namespace std;

struct LocalPathPoint {
    double x;
    double y;
    double theta;
    double kappa;

    double s;
};

typedef enum {
    smoother_no_global_path,
    smoother_failed,
    planner_no_fesiable_region,
    planner_success,
    planner_optimize_fail,
    planner_optimize_warning,
    planner_reach_goal
} PlannerStatus;

typedef enum {
    start,
    splice,
    await,
    end
} PlannerState;

class BypassObstaclePlanner { // 输入 : 车辆位姿 && 代价地图 && 参考路径
    public:
        BypassObstaclePlanner(SaveData* p_savedata);
        virtual ~BypassObstaclePlanner();

        virtual PlannerStatus GetLocalPath(
                deque<LocalPathPoint>& local_path) = 0;
        virtual void Reset() = 0;

        bool SetCurvePoints(const deque<CurvePoint>& curve_points); // 写入粗糙轨迹点信息序列
        void SetRobotPose(const RobotPose& pose); // 写入机器人位姿信息

    protected:
        virtual void SetCostMap(MatrixXd& cost_map) = 0; // debug
        virtual void GetCostMap() = 0;

        virtual void SaveLog() = 0; // 保存日志
        virtual void DisplayInfo() = 0;
        
        QpSplineSmoother *curve_smoother_ptr_;
        // NaviCostmap map_ptr_;
        // CollisionChecker collision_check_prt_;

        deque<SmoothLinePoint> global_path_smoothed_; // 平滑后的全局路径点信息序列
        deque<LocalPathPoint> local_path; // 平滑后局部轨迹点信息序列
        
        MatrixXd occupy_map_;
        RobotPose pose_;

        double robot_len_;
        double robot_wid_;
        double dis_margin_;

        SaveData *p_savedata_; // debug
};

class PiecewiseJerkPathOptimization : public BypassObstaclePlanner {
    public:
        PiecewiseJerkPathOptimization(SaveData* p_savedata);
        virtual ~PiecewiseJerkPathOptimization() = default;
        virtual PlannerStatus GetLocalPath(
                deque<LocalPathPoint>& local_path);
        virtual void Reset();

    protected:
        virtual void SetCostMap(MatrixXd& cost_map); // debug
        virtual void GetCostMap();
            
    private:
        virtual void SaveLog(); // 保存日志
        virtual void DisplayInfo();

        int planning_times_;

        double running_time_; // debug
};
