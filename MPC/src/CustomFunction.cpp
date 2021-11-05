#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <cassert>
#include <math.h>
#include <cassert>

#include "CustomFunction.h"

using namespace std;

namespace CustomFunction
{

double norm(const vector<double> &x)
{
    double val = 0.0;

    for (auto elem: x) {
        val += elem * elem;
    }

    return sqrt(val);
}

vector<double> operator-(const vector<double> &x, const vector<double> &y)
{
    assert(x.size() == y.size());

    vector<double> tmp;

    for (size_t i = 0; i < x.size(); ++i) {
        tmp.push_back(x[i] - y[i]);
    }

    return tmp;
}

double operator*(const vector<double> &x, const vector<double> &y)
{
    assert(x.size() == y.size());

    double sum = 0.0;

    for (size_t i = 0; i < x.size(); ++i) {
        sum += x[i] * y[i];
    }

    return sum;
}

vector<double> operator^(const vector<double> &x, const vector<double> &y)
{
    assert(x.size() == y.size() && x.size() == 3);

    return vector<double> { x[1] * y[2] - x[2] * y[1],
                            x[2] * y[0] - x[0] * y[2],
                            x[0] * y[1] - x[1] * y[0] };
}

double twoDCrossProd(const vector<double> &x, const vector<double> &y)
{
    return x[0] * y[1] - x[1] * y[0];
}

double angle(const vector<double> &x, const vector<double> &y)
{
    return acos(x * y / norm(x) / norm(y));
}

double distance(
        const vector<double> &x0, const vector<double> &x1,
        const vector<double> &x2)
{
    return twoDCrossProd(x1 - x0, x2 - x0) / norm(x1 - x2);
}

double AngleLimiting(double x)
{
    while (x > M_PI) {
        x = x - 2.0 * M_PI;
    }

    while (x <= -1.0 * M_PI) {
        x = x + 2.0 * M_PI;
    }

    return x;
}

void txt_to_vectordouble(vector<vector<double>>& res, string pathname)
{
    string string_;

    ifstream ReadFile;
    ReadFile.open(pathname, ios::in);

    if (ReadFile.fail()) {
        cout << "[error] failed to open : " << pathname << endl;
    } else {
        while (getline(ReadFile, string_)) {
            istringstream is(string_);

            double data_;
            vector<double> temp;

            while (!is.eof()) {
                is >> data_;
                temp.push_back(data_);
            }

            res.push_back(temp);
            temp.clear();
            string_.clear();
        }
    }

    ReadFile.close();
}

double interp_linear(vector<double> x, vector<double> y, double x0)
{
    long unsigned int i = x.size() - 1;
    long unsigned int j;

    if (x0 < x.at(0)) {
        x0 = x.at(0) + 0.0001;
    }
    else if (x0 > x.at(i)) {
        x0 = x.at(i) - 0.0001;
    }

    for (j = 0; j < i; j++) {
        double temp = (x0 - x.at(j)) * (x0 - x.at(j + 1));

        if (temp <= 0.0) {
            break;
        }
    }

    double y_result =
            y.at(j) + (x0 - x.at(j)) * (y.at(j + 1) -
            y.at(j)) / (x.at(j + 1) - x.at(j));

    return y_result;
}

template <typename T> T sgn(T val)
{
    T res;

    if(val > T(0)) {
        res = T(1);
    } else if (val == T(0)) {
        res = T(0);
    } else {
        res = T(-1);
    }

    return res;
}

Eigen::MatrixXd KroneckerProduct(Eigen::MatrixXd A, Eigen::MatrixXd B)
{
    int r_A, c_A, r_B, c_B, r_res, c_res;

    r_A = A.rows();
    c_A = A.cols();
    r_B = B.rows();
    c_B = B.cols();
    r_res = r_A * r_B;
    c_res = c_A * c_B;

    Eigen::MatrixXd res(r_res, c_res);
    for (int i = 0; i < r_A; i++) {
        for (int j = 0; j < c_A; j++) {
            res.block(i * r_B, j * c_B, r_B, c_B) = A(i, j) * B;
        }
    }

    return res;
}

}