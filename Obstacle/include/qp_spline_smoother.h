#pragma once

#include <vector>
#include <eigen3/Eigen/Eigen>
#include <memory>
#include <queue>

#include "InformationFormat.h"
#include "SaveData.h"

using namespace std;
using namespace Eigen;

struct RobotPose {
    double x;
    double y;
    double yaw;
};

struct CurvePoints {
    double x;
    double y;
    double theta;
    double kappa;
};

struct LocalTrajPoints {
    double x;
    double y;
    double theta;
    double kappa;

    double s; // 增量求解与拼接使用 && s-t速度规划使用
};

struct SamplingPoints {
    double x;
    double y;
    double theta;

    double s;
};

typedef enum {
    success,
    wait,
    finish,
    warning_optimize,
    fail_optimize,
    fail_no_global_path
} SmootherStatus;

typedef enum {
    init,
    splicing,
    waiting,
    finished
} SmootherState;

class CurveSmoother {
    public:
        CurveSmoother(SaveData* p_savedata);
        virtual ~CurveSmoother() {};

        virtual SmootherStatus GetSmoothCurve(
                deque<LocalTrajPoints>& smooth_line_points, RobotPose& pose) = 0;

        bool SetCurvePoints(const deque<CurvePoints>& curve_points);

        virtual void Reset() = 0;

        void SetRobotPose(RobotPose& pose);

    protected:
        virtual void SaveLog() = 0;

        deque<CurvePoints> curve_points_; // queue
        vector<double> accumulative_length_; // deque
        vector<double> pos_x_;
        vector<double> pos_y_;
        vector<double> pos_theta_;
        deque<LocalTrajPoints> smooth_line_; // deque

        RobotPose robot_pose_;

        SaveData *p_savedata_; // debug
};

class QpSplineSmoother : public CurveSmoother {
    public:
        QpSplineSmoother(SaveData* p_savedata);
        virtual ~QpSplineSmoother() {};

        virtual SmootherStatus GetSmoothCurve(
                deque<LocalTrajPoints>& smooth_line_points, RobotPose& pose);

        virtual void Reset();
        
        void InfoShow();

    private:
        void SmootherParaCfg();
        void CalSamplingPoints();
        void CalSplicingPoint();
        void CalObjectiveFunc(MatrixXd& matrix_h, VectorXd& matrix_f);
        void CalEqualityConstraint(
                MatrixXd& matrix_a_equ, MatrixXd& matrix_b_equ);
        void CalInequalityConstraint(
                MatrixXd& matrix_a_inequ, MatrixXd& matrix_b_inequ);
        void CalSmoothTraj(VectorXd& poly_coefficient);
        
        int  OptimizationSolver(
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

        double InterpLinear(vector<double>& x, vector<double>& y, double x0);
        double QuickInterpLinear(vector<double>& x, vector<double>& y, double x0, int& idx_init);
        double Norm(const vector<double> & x);
        double VecDotMultip(const vector<double> &x, const vector<double> &y);
        int Double2Int(double var) { return (int)(var + 0.5); }

        bool QuarterTurnExamine(vector<double>& point_pos,double len_examined);
        int FindNearestPoint(
                vector<double> position, deque<CurvePoints>& curve_points);
        bool GoalPointExamine(
                vector<double>& point_pos,double len_examined, double& dis2goal);

        virtual void SaveLog();

        void Txt2Vector(deque<CurvePoints>& res, string pathname); // debug

        double len_line_ = 6.0;
        double len_fragment_ = 1.0;
        double interval_sampling_ = 0.2;
        double interval_sample_ = 0.01;
        double ub_line_len_ = 10.0;

        vector<SamplingPoints> sampling_points_;

        double weight_acc_x_  = 0.01;
        double weight_jerk_x_ = 0.02;
        double weight_pos_x_  = 2000.0;
        double weight_acc_y_  = 0.01;
        double weight_jerk_y_ = 0.02;
        double weight_pos_y_  = 2000.0;

        double max_pos_err_x_ = 0.02;
        double max_pos_err_y_ = 0.02;

        int nums_fragments_ = Double2Int(len_line_ / len_fragment_);
        int nums_in_fragment_ = Double2Int(len_fragment_ / interval_sampling_);

        c_int osqp_max_iteration_ = 200;
        c_float osqp_eps_abs_ = 0.001;

        int times_osqp_failed_ = 0;
        int max_times_failed_ = 3;

        int times_smoothing_ = 0;

        double running_time_ = 0.0;

        LocalTrajPoints sample_start_point_;

        SmootherState smoother_state_ = init;
};

// todo : 画图 + 拼接剪裁 + s
// todo : 拼接剪裁测试 ParaCfg()测试 折线转弯测试 goal_point测试
// todo : review
// todo : 分段加加速度优化架构

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
