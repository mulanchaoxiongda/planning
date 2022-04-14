#pragma once

#include <vector>
#include <eigen3/Eigen/Eigen>
#include <memory>
#include <queue>

#include "InformationFormat.h"
#include "SaveData.h"

using namespace std;
using namespace Eigen;

struct CurvePoints {
    double x;
    double y;
    double theta;
};

struct SamplingPoints {
    double x;
    double y;
    double theta;

    double s;
};

typedef enum {
    success,
    fail
} SmootherStatus;

class CurveSmoother {
    public:
        CurveSmoother(RobotModel* p_RobotModel, SaveData* p_savedata);
        virtual ~CurveSmoother() {};

        virtual SmootherStatus GetSmoothCurve(vector<CurvePoints>& smooth_line_points) = 0;

        bool SetCurvePoints(const vector<CurvePoints>& curve_points);

    protected:
        vector<CurvePoints> curve_points_; // queue
        vector<double> accumulative_length_; // deque
        vector<double> pos_x_;
        vector<double> pos_y_;
        vector<double> pos_theta_;
        vector<CurvePoints> smooth_line_; // deque

        SaveData *p_savedata_; // debug
        RobotModel *p_RobotModel_; // debug
};

class QpSplineSmoother : public CurveSmoother {
    public:
        QpSplineSmoother(RobotModel* p_RobotModel, SaveData* p_savedata);
        //QpSplineSmoother(RobotModel* p_RobotModel, SaveData* p_savedata) : CurveSmoother(p_RobotModel, p_savedata) {};
        virtual ~QpSplineSmoother() {};

        SmootherStatus GetSmoothCurve(vector<CurvePoints>& smooth_line_points);

    private:
        void CalSamplingPoints();
        void CalObjectiveFunc(MatrixXd& matrix_h, VectorXd& matrix_f);
        void CalEqualityConstraint(MatrixXd& matrix_a_equ, MatrixXd& matrix_b_equ);
        void CalInequalityConstraint(MatrixXd& matrix_a_inequ, MatrixXd& matrix_b_inequ);
        void CalSmoothTraj(VectorXd& poly_coefficient);
        
        int  OptimizationSolver(VectorXd &optimal_solution, MatrixXd matrix_p, VectorXd vector_q, MatrixXd matrix_Ac, VectorXd vector_l, VectorXd vector_u, c_int max_iteration, c_float eps_abs);
        void MatrixToCCS(MatrixXd matrix_a, vector<c_float> *sm_x, c_int &sm_nnz, vector<c_int> *sm_i, vector<c_int> *sm_p);
        
        template <typename T>
        T *CopyData(const std::vector<T> &vec) {
            // T *data = new T[vec.size()];
            T *data = (T*)c_malloc(vec.size()*sizeof(T));
            memcpy(data, vec.data(), sizeof(T) * vec.size());
            return data;
        }

        double InterpLinear(vector<double>& x, vector<double>& y, double x0);

        void Txt2Vector(vector<CurvePoints>& res, string pathname); // debug

        double len_line_ = 6.0;
        double len_fragment_ = 1.0;
        double interval_sampling_ = 0.2;

        vector<SamplingPoints> sampling_points_;

        double weight_acc_x_  = 0.01;
        double weight_jerk_x_ = 0.02;
        double weight_pos_x_  = 2000.0;
        double weight_acc_y_  = 0.01;
        double weight_jerk_y_ = 0.02;
        double weight_pos_y_  = 2000.0;

        double max_pos_err_x_ = 0.01;
        double max_pos_err_y_ = 0.01;

        int nums_fragments_ = len_line_ / len_fragment_;
        int nums_in_fragment_ = len_fragment_ / interval_sampling_;
};
