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

        virtual void LoadOccupyMap(string pathname) = 0; // debug

    protected:
        virtual void LoadOccupyMap() = 0;

        virtual void SaveLog() = 0; // 保存日志
        
        QpSplineSmoother *curve_smoother_ptr_;
        // NaviCostmap map_ptr_; // todo : 构造函数初始化列表初始化
        // CollisionChecker collision_check_prt_; // todo : 构造函数初始化列表初始化

        deque<SmoothLinePoint> global_path_smoothed_; // 平滑后的全局路径点信息序列
        deque<LocalPathPoint> local_path_; // 平滑后局部轨迹点信息序列
        
        MatrixXd occupy_map_;
        RobotPose robot_pose_;

        double robot_len_;
        double robot_wid_;
        double dis_margin_;

        double map_resolution_;

        PlannerState planner_state_;

        SaveData *p_savedata_; // debug
};

class PiecewiseJerkPathOptimization : public BypassObstaclePlanner {
    public:
        PiecewiseJerkPathOptimization(SaveData* p_savedata);
        virtual ~PiecewiseJerkPathOptimization() = default;
        virtual PlannerStatus GetLocalPath(
                deque<LocalPathPoint>& local_path);
        virtual void Reset();

        virtual void LoadOccupyMap(string pathname); // debug

    protected:
        virtual void LoadOccupyMap(); // 加载占用栅格图

    private:
        // void SmootherParaCfg(); // 参数配置
        // bool QuarterTurnExamine(
        //         const vector<double>& point_pos, const double& len_examined);
        // int  FindNearestPoint(
        //         const vector<double>& position, const deque<CurvePoint>& curve_points);
        // int  FindNearestPoint(
        //         const vector<double>& position, const deque<SmoothLinePoint>& smooth_traj);
        // bool GoalPointExamine(
        //         const vector<double>& point_pos, const double& len_examined,
        //         double& dis2goal);

        // void CalSamplingPoints(); // 路径点序列增量求解、拼接与剪切
        // void ShearCutTraj();
        
        // void CalObjectiveFunc(MatrixXd& matrix_h, VectorXd& matrix_f) const; // 分段五次多项式拟合优化问题构建
        // void CalEqualityConstraint(
        //         MatrixXd& matrix_a_equ, MatrixXd& matrix_b_equ) const;
        // void CalInequalityConstraint(
        //         MatrixXd& matrix_a_inequ, MatrixXd& matrix_b_inequ) const;
        // void CalSmoothTraj(
        //         const VectorXd& poly_coefficient,
        //         deque<SmoothLinePoint>& smooth_line);

        // c_int OptimizationSolver( // 二次规划问题求解
        //         VectorXd &optimal_solution, MatrixXd matrix_p,
        //         VectorXd vector_q, MatrixXd matrix_Ac,
        //         VectorXd vector_l, VectorXd vector_u,
        //         c_int max_iteration, c_float eps_abs);
        // void MatrixToCCS(
        //         MatrixXd matrix_a, vector<c_float> *sm_x, c_int &sm_nnz,
        //         vector<c_int> *sm_i, vector<c_int> *sm_p);   
        // template <typename T>
        // T *CopyData(const std::vector<T> &vec) {
        //     T *data = (T*)c_malloc(vec.size()*sizeof(T));
        //     memcpy(data, vec.data(), sizeof(T) * vec.size());
        //     return data;
        // }

        // double InterpLinear( // 数学计算函数
        //         const vector<double>& x, const vector<double>& y, double x0) const;
        // double FastInterpLinear(
        //         const vector<double>& x, const vector<double>& y, double x0, int& idx_init) const;
        // double Norm(const vector<double> & x) const;
        // double VecDotMultip(const vector<double> &x, const vector<double> &y) const;
        // int Double2Int(double var) { return (int)(var + 0.5); }

        virtual void SaveLog(); // 保存日志
        void DisplayInfo() const; // 打印信息

        void txt_to_vectordouble(vector<vector<bool>>& res, string pathname); // debug

        int planning_times_;

        double running_time_; // debug
};


// todo : 按照自己理解，架构流程函数，并实现非优化相关函数功能
// todo : 个人理解 + 前期笔记 + 路径平滑代码
