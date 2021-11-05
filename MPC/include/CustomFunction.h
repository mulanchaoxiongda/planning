#pragma once

#include <vector>
#include <eigen3/Eigen/Eigen>

using namespace std;

namespace CustomFunction
{

//向量求模
double norm(const vector<double> & x);

//向量求差
vector<double> operator-(const vector<double> &x, const vector<double> &y);

//向量点乘
double operator*(const vector<double> &x, const vector<double> &y);

//三维向量叉乘
vector<double> operator^(const vector<double> &x, const vector<double> &y);

//二维向量叉乘
double twoDCrossProd(const vector<double> &x, const vector<double> &y);

//求向量夹角
double angle(const vector<double> &x, const vector<double> &y);

//求点x0到直线x1x2距离
double distance(const vector<double> &x0, const vector<double> &x1,
                const vector<double> &x2);

//角度值限制在-pi到pi
double AngleLimiting(double x);

//从.txt文件读入数据，存入二维vector
void txt_to_vectordouble(vector<vector<double>>& res, string pathname);

//一维线性插值
double interp_linear(vector<double> x, vector<double> y, double x0);

//符号函数
template <typename T> T sgn(T val);

//矩阵直积
Eigen::MatrixXd KroneckerProduct(Eigen::MatrixXd A, Eigen::MatrixXd B);

};