#include <math.h>
#include <algorithm>
#include <string.h>
#include <fstream>
#include <iostream>
#include <string>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <cassert>
#include <iomanip>
#include <memory>
#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>
#include <iostream>
#include <sys/time.h>

#include "TrackingAlgorithm.h"

TrackingAlgorithm::TrackingAlgorithm(RobotModel *p_RobotModel, SaveData *p_savedata)
{
    p_RobotModel_ = p_RobotModel;
    p_savedata_ = p_savedata;
}

void TrackingAlgorithm::ReadInTrajPoints()
{
    string string_temp;

    ifstream ReadFile;
    ReadFile.open("../data/TrajectoryPoints.txt", ios::in);

    if (ReadFile.fail()) {
        cout << "[error] failed to open TrajectoryPoints.txt" << endl;
    } else {
        while (getline(ReadFile, string_temp)) {
            istringstream is(string_temp);

            double data;

            vector<double> Temp;
            TrajPoint temp;

            while (!is.eof()) {
                is>>data;
                Temp.push_back(data);
            }

            temp.x_ref   = Temp.at(0);
            temp.y_ref   = Temp.at(1);
            temp.yaw_ref = Temp.at(2);
            temp.v_ref   = Temp.at(3);
            temp.w_ref   = Temp.at(4);
            temp.t_ref   = Temp.at(5);

            trajectory_points_.push_back(temp);

            Temp.clear();
            string_temp.clear();
        }
    }

    ReadFile.close();
    cout << "[INFO] read in reference trajectory points successfully !" << endl;
}

void TrackingAlgorithm::ReadInTrajPoints(vector<TrajPoint> local_traj_points)
{
    trajectory_points_.assign(local_traj_points.begin(), local_traj_points.end());
}

void TrackingAlgorithm::GetSensorInfo()
{
    memcpy(&sensor_info_, &p_RobotModel_->motion_state_, sizeof(SensorInfo));
}

// 参考点轨迹的时标需要大于控制算法运行时刻
void TrackingAlgorithm::FindRefPoint(
        vector<TrajPoint> &trajectory_points, SensorInfo &sensor_info)
{
    vector<double> RelativeTime;
    vector<double> fabs_RelativeTime;

    int SizeOfRefTraj = trajectory_points.size();

    if (SizeOfRefTraj <= 1 ) {
        cout << "[error] local reference trajectory has only " << SizeOfRefTraj << " points " << endl << endl;
    }

    for (int i = 0; i < SizeOfRefTraj; i++) {
        double delta_t, fabs_delta_t;

        delta_t = sensor_info.t - trajectory_points.at(i).t_ref;
        fabs_delta_t = -1.0 * fabs(delta_t);

        RelativeTime.push_back(delta_t);
        fabs_RelativeTime.push_back(fabs_delta_t);
    }

    vector<double>::iterator biggest =
            max_element(begin(fabs_RelativeTime), end(fabs_RelativeTime));

    int _ID, ID_RefPoint;

    _ID = distance(begin(fabs_RelativeTime), biggest);

    if (RelativeTime.at(_ID) <= 0.0) {
        ID_RefPoint = _ID - 1;

        if(ID_RefPoint == -1) {
            ID_RefPoint = 0;
        }
    } else {
        ID_RefPoint = _ID;

        if (ID_RefPoint == (SizeOfRefTraj - 1)) {
            ID_RefPoint = SizeOfRefTraj - 2;
        }
    }

    double dis1, dis2, dis_total;

    dis1 = fabs_RelativeTime.at(ID_RefPoint);
    dis2 = fabs_RelativeTime.at(ID_RefPoint + 1);

    dis_total = dis1 + dis2;

    reference_point_.x =
            ( trajectory_points.at(ID_RefPoint).x_ref * dis2 +
              trajectory_points.at(ID_RefPoint + 1).x_ref * dis1 ) /
              dis_total;
    reference_point_.y =
            ( trajectory_points.at(ID_RefPoint).y_ref * dis2 +
              trajectory_points.at(ID_RefPoint + 1).y_ref * dis1 ) /
              dis_total;
    reference_point_.yaw =
            ( trajectory_points.at(ID_RefPoint).yaw_ref * dis2 +
              trajectory_points.at(ID_RefPoint + 1).yaw_ref * dis1 ) /
              dis_total;
    reference_point_.v =
            ( trajectory_points.at(ID_RefPoint).v_ref * dis2 +
              trajectory_points.at(ID_RefPoint + 1).v_ref * dis1 ) /
              dis_total;
    reference_point_.w =
            ( trajectory_points.at(ID_RefPoint).w_ref * dis2 +
              trajectory_points.at(ID_RefPoint + 1).w_ref * dis1 ) /
              dis_total;
    reference_point_.t =
            ( trajectory_points.at(ID_RefPoint).t_ref * dis2 +
              trajectory_points.at(ID_RefPoint + 1).t_ref * dis1 ) /
              dis_total;

    p_savedata_->file << "[reference_point] "
                      << " Time "    << reference_point_.t
                      << " x_ref "   << reference_point_.x
                      << " y_ref "   << reference_point_.y
                      << " yaw_ref " << reference_point_.yaw
                      << " v_ref "   << reference_point_.v
                      << " v_ref "   << reference_point_.w
                      << " t "       << reference_point_.t << endl;
}

