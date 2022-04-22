#pragma once

#include <deque>
#include <vector>
#include <eigen3/Eigen/Eigen>

#include "InformationFormat.h"
#include "SaveData.h"
#include "curve_smoother.h"

using namespace std;

struct ExpandInterval {
    int idx_start;
    int idx_end;
    double lmax_min;
    double lmin_max;
};

struct FesiableRegion {
    double x;
    double y;
    double theta;
    double kappa;

    double s;
    double l_min;
    double l_max;
};

struct BasePoint {
    double x;
    double y;
    double theta;
    double kappa;

    double s;
    double l_min;
    double l_max;
};

struct LocalPathPoint {
    double s_frenet; // todo : 调试成熟后可删除s && l
    double l_frenet;
    double x;
    double y;
    double theta;
    double kappa;

    double s;
};

typedef enum {
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
        SmootherStatus smoother_status_;

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
        void SmootherParaCfg(); // 参数配置

        bool SmoothGlobalPath(); // 平滑全局路径
        bool CalPathFesiableRegion(); // 栅格障碍信息映射到s-l系求可行域
        void SamplingBasePoints(); // 采点

        void CalObjectiveFunc(MatrixXd& matrix_h, VectorXd& matrix_f) const; // 分段五次多项式拟合优化问题构建
        void CalEqualityConstraint(
                MatrixXd& matrix_a_equ, MatrixXd& matrix_b_equ) const;
        void CalInequalityConstraint(
                MatrixXd& matrix_a_inequ, MatrixXd& matrix_b_inequ) const;
        void CalLocalPath(
                const VectorXd& poly_coefficient,
                deque<SmoothLinePoint>& smooth_line);
        void ShearCutPath();

        void CalGlobalSystemPath();

        bool CalPointFesiableRegion(
                double& l_min, double& l_max, const SmoothLinePoint point_info);
        bool ExpandObstacles();
        bool ExpandObstacle(
                const ExpandInterval expand_interval,
                const double& modified_road_half_wid);
        vector<int> CalPathLenWithinMap(); // 返回值 : global_paht_smoothed_位于栅格图内的起点和边界点的索引值
        int  FindNearestPoint(
                const vector<double>& position,
                const deque<SmoothLinePoint>& smooth_traj);
        vector<double> FindMatchPoint(const double& x, const double& y);
        bool GoalPointExamine(
                const vector<double>& point_pos, const double& len_examined,
                double& dis2goal);

        c_int OptimizationSolver( // 二次规划问题求解
                VectorXd &optimal_solution, MatrixXd matrix_p,
                VectorXd vector_q, MatrixXd matrix_Ac,
                VectorXd vector_l, VectorXd vector_u,
                c_int max_iteration, c_float eps_abs);
        void MatrixToCCS(
                MatrixXd matrix_a, vector<c_float> *sm_x, c_int &sm_nnz,
                vector<c_int> *sm_i, vector<c_int> *sm_p);   
        template <typename T>
        T *CopyData(const std::vector<T> &vec) {
            T *data = (T*)c_malloc(vec.size()*sizeof(T));
            memcpy(data, vec.data(), sizeof(T) * vec.size());
            return data;
        }

        double InterpLinear( // 数学计算函数
                const vector<double>& x, const vector<double>& y, double x0) const;
        double FastInterpLinear(
                const vector<double>& x, const vector<double>& y, double x0, int& idx_init) const;
        double Norm(const vector<double> & x) const;
        double VecDotMultip(const vector<double> &x, const vector<double> &y) const;
        int Double2Int(double var) { return (int)(var + 0.5); }
        double MaxDouble(double& x, double& y) {return x > y ? x : y;}
        double MinDouble(double& x, double& y) {return x < y ? x : y;}

        virtual void SaveLog(); // 保存日志
        void DisplayInfo() const; // 打印信息

        void txt_to_vectordouble(vector<vector<bool>>& res, string pathname); // debug

        deque<FesiableRegion> fesiable_region_;
        deque<BasePoint> base_points_; // todo : 注意s值的对齐，如果对不齐，6m结束规划不可行
        vector<ExpandInterval> expand_intervals_;

        double len_ref_s_;
        
        int planning_times_;

    private: // debug
        bool IsPointInMap(double x, double y, double x_agv, double y_agv);
        bool IsPointOccupied(const double x, const double y);

        double running_time_;

        const double pi_half_ = 3.1415926536 / 2.0;
        const double pi_ = 3.1415926536;
};

// todo : 按照自己理解，架构流程函数，并实现非优化相关函数功能 ： 
//        个人理解（总纲、细节） + 
//        前期笔记(总纲) + 
//        路径平滑代码（优化部分 && 业务逻辑 && 构造、重置函数）

// todo : 初步构思，规划轨迹长度为8m，smoother_finish时，smooth_line_.back（）硬约束，其他时候软约束
//        规划轨迹小于6m（地图参数）且smoother_finished且含终点硬约束，则planner_finished
//        强弱规划周期保证帧间轨迹的时间一致性，无增量规划、在线拼接与剪裁的概念
//        建议matlab仿真完毕后，确定参数再定方案
//        如果6m规划轨迹的长度可保证平滑绕障，建议取6m，可降低逻辑复杂度
