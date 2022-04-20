#pragma once

#include <vector>
#include <eigen3/Eigen/Eigen>
#include <memory>
#include <queue>

#include "InformationFormat.h"
#include "SaveData.h"

using namespace std;
using namespace Eigen;

struct RobotPose { // 机器人位姿
    double x;
    double y;
    double yaw;
};

struct CurvePoint { // 读入的粗糙轨迹信息
    double x;
    double y;
    double theta;
};

struct SmoothLinePoint { // 输出的平滑后轨迹信息
    double x;
    double y;
    double theta;
    double kappa;

    double s; // s-t速度规划使用
};

struct SamplingPoint { // 采点信息
    double x;
    double y;
    double theta;
};

typedef enum { // 对外输出的平滑器状态
    success,
    wait,
    finish,
    warning_optimize,
    fail_optimize,
    fail_no_global_path
} SmootherStatus;

typedef enum { // 内部参数配置使用的平滑器状态
    init,
    splicing,
    waiting,
    finished
} SmootherState;

class CurveSmoother { // 平滑器父类
    public:
        CurveSmoother(SaveData* p_savedata);
        virtual ~CurveSmoother() {};

        virtual SmootherStatus GetSmoothCurve( // 核心函数 : 计算平滑后轨迹点信息序列，返回求解器状态
                deque<SmoothLinePoint>& smooth_line_points) = 0;
        virtual void Reset() = 0;

        bool SetCurvePoints(const deque<CurvePoint>& curve_points); // 写入粗糙轨迹点信息序列
        void SetRobotPose(const RobotPose& pose); // 写入机器人位姿信息

    protected:
        virtual void SaveLog() = 0; // 保存日志

        deque<CurvePoint> curve_points_; // 粗糙轨迹点信息序列
        deque<SmoothLinePoint> smooth_line_; // 平滑后轨迹点信息序列

        RobotPose robot_pose_; // 机器人位姿

        SaveData *p_savedata_; // debug
};

class QpSplineSmoother : public CurveSmoother {
    public:
        QpSplineSmoother() = delete;
        QpSplineSmoother(SaveData* p_savedata);
        virtual ~QpSplineSmoother() = default;

        virtual SmootherStatus GetSmoothCurve( // 核心函数 : 计算平滑后轨迹点信息序列，返回求解器状态
                deque<SmoothLinePoint>& smooth_line_points);
        virtual void Reset();

        void PrintInfo() const; // 打印运行信息

        void Txt2Vector(deque<CurvePoint>& res, const string& pathname); // debug

    private:
        void SmootherParaCfg(); // 参数配置
        bool QuarterTurnExamine(
                const vector<double>& point_pos, const double& len_examined);
        int  FindNearestPoint(
                const vector<double>& position, const deque<CurvePoint>& curve_points);
        int  FindNearestPoint(
                const vector<double>& position, const deque<SmoothLinePoint>& smooth_traj);
        bool GoalPointExamine(
                const vector<double>& point_pos, const double& len_examined,
                double& dis2goal);

        void CalSamplingPoints(); // 路径点序列增量求解、拼接与剪切
        void ShearCutTraj();
        
        void CalObjectiveFunc(MatrixXd& matrix_h, VectorXd& matrix_f) const; // 分段五次多项式拟合优化问题构建
        void CalEqualityConstraint(
                MatrixXd& matrix_a_equ, MatrixXd& matrix_b_equ) const;
        void CalInequalityConstraint(
                MatrixXd& matrix_a_inequ, MatrixXd& matrix_b_inequ) const;
        void CalSmoothTraj(const VectorXd& poly_coefficient);

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

        virtual void SaveLog(); // 保存日志

        double len_line_; // 采点相关参数
        double len_init_;
        double len_increment_;
        double len_fragment_;
        double interval_sampling_;
        double interval_sample_;
        double ub_line_len_;
        int nums_fragments_;
        int nums_in_fragment_;

        double max_turn_angle_; // 折线转弯判据

        double weight_acc_x_; // 二次规划软、硬约束相关参数
        double weight_jerk_x_;
        double weight_pos_x_;
        double weight_acc_y_;
        double weight_jerk_y_;
        double weight_pos_y_;
        double max_pos_err_x_;
        double max_pos_err_y_;
        double max_pos_err_init_;
        double pos_err_incre_;

        SmootherState smoother_state_; // 平滑器状态相关参数
        int times_osqp_failed_;
        int max_times_failed_;
        int times_smoothing_;

        int osqp_solver_exitflag_; // osqp求解器相关参数
        c_int osqp_max_iteration_;
        c_float osqp_eps_abs_;
        
        double running_time_; // debug

        vector<SamplingPoint> sampling_points_; // 采点信息序列
        SamplingPoint sample_start_point_; // 采点起始点
};





// todo : 画图
// todo : 拼接剪裁测试 ParaCfg()测试 折线转弯测试 goal_point测试
// todo : 默认global_paht点列增序方向就是机器人期望行驶方向

// experience_01 : 处理Frenet系局部路径转Global系局部路径奇异性，构思以下两种种方案：
//                 [方案一] 机器人转弯半径小，为了防止障碍物映射到s-l坐标系时在内弧侧出现奇异性，参考线必须平滑路径全长，l不超出内弧半径范围；
//                         算力优化：记录s-l系各采样基准点的(s_base_i, stheta_base_i)信息，以空间换时间；
//                 [方案二] 参考线平滑只需要覆盖max_times_failed_ + 1帧对应的弧段即可，如果存在局部路径后端优化器，同样仅优化并输出max_times_failed_ + 1帧对应的弧段即可；
//                         算力优化: 1.静态障碍物映射到s-l系(EM Planner/分段加加速度优化算法)，max_times_failed_ + 1帧对应弧段的参考线和局部路径(s_i_opti, l_i_opti)均需要平滑；
//                                  2.静态障碍物映射到s-l系(EM Planner/分段加加速度优化算法)，仅平滑局部路径(x_i_opti, y_i_opti);
//                                  3.静态障碍物映射到global系(Hybrid Astar)，仅平滑局部路径即可；
//                                  4.记录s-l系各采样基准点的(s_base_i, stheta_base_i)信息，以空间换时间，同时避免了求解优化后(s_i_opti, l_i_opti)点的s-l系基准点的奇异性问题；
//                                  5.建议s-l系各采样点的l_i不超出相应基准点(s_base_i, 0)的内弧半径范围；
// experience_02 : 增量平滑 + 拼接方案：选取上帧终点作为拼接帧起点，平滑增量采样点后与上帧首尾截断后剩余弧线拼接
//                 [实现] 1.CalSamplingPoints()实现了首帧采点，可重载实现拼接帧采点；截断与拼接可通过重载或修改现有函数实现；
//                       2.上帧终点作为拼接帧起点 + 末端大角度折线转弯时多平滑2m长度，两者结合保证折线转弯平滑路径品质。
// experience_03 : osqp warm_start
// experience_04 : 折线转弯 放宽位置误差硬约束，依靠位置误差软约束提高位置拟合精度 增大平滑软约束权重
