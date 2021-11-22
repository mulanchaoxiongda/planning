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

    ifstream read_file;
    read_file.open("../data/TrajectoryPoints.txt", ios::in);

    if (read_file.fail()) {
        cout << "[error] failed to open TrajectoryPoints.txt" << endl;
    } else {
        while (getline(read_file, string_temp)) {
            istringstream is(string_temp);

            double data;

            vector<double> temp_container;
            TrajPoint temp;

            while (!is.eof()) {
                is>>data;
                temp_container.push_back(data);
            }

            temp.x_ref   = temp_container.at(0);
            temp.y_ref   = temp_container.at(1);
            temp.yaw_ref = temp_container.at(2);
            temp.v_ref   = temp_container.at(3);
            temp.w_ref   = temp_container.at(4);
            temp.t_ref   = temp_container.at(5);

            trajectory_points_.push_back(temp);

            temp_container.clear();
            string_temp.clear();
        }
    }

    read_file.close();
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
    vector<double> relative_time;
    vector<double> fabs_relative_time;

    int SizeOfRefTraj = trajectory_points.size();

    if (SizeOfRefTraj <= 1 ) {
        cout << "[error] local reference trajectory has only " << SizeOfRefTraj << " points " << endl << endl;
    }

    for (int i = 0; i < SizeOfRefTraj; i++) {
        double delta_t, fabs_delta_t;

        delta_t = sensor_info.t - trajectory_points.at(i).t_ref;
        fabs_delta_t = -1.0 * fabs(delta_t);

        relative_time.push_back(delta_t);
        fabs_relative_time.push_back(fabs_delta_t);
    }

    vector<double>::iterator biggest =
            max_element(begin(fabs_relative_time), end(fabs_relative_time));

    int ref_point_index;

    ref_point_index = distance(begin(fabs_relative_time), biggest);

    if (relative_time.at(ref_point_index) <= 0.0) {
        ref_point_index = ref_point_index - 1;

        if(ref_point_index == -1) {
            ref_point_index = 0;
        }
    } else {
        if (ref_point_index == (SizeOfRefTraj - 1)) {
            ref_point_index = SizeOfRefTraj - 2;
        }
    }

    double dis1, dis2, dis_total;

    dis1 = fabs_relative_time.at(ref_point_index);
    dis2 = fabs_relative_time.at(ref_point_index + 1);

    dis_total = dis1 + dis2;

    reference_point_.t =
            ( trajectory_points.at(ref_point_index).t_ref * dis2 +
              trajectory_points.at(ref_point_index + 1).t_ref * dis1 ) /
              dis_total;

    reference_point_.v =
            ( trajectory_points.at(ref_point_index).v_ref * dis2 +
              trajectory_points.at(ref_point_index + 1).v_ref * dis1 ) /
              dis_total;

    cout << trajectory_points.at(ref_point_index).v_ref * dis2 << endl;
    cout << trajectory_points.at(ref_point_index + 1).v_ref * dis1 << endl;
    cout << dis_total << endl;

    cout << "ref_v  ref_v  ref_v  ref_v" << reference_point_.v << endl;
    
    reference_point_.w =
            ( trajectory_points.at(ref_point_index).w_ref * dis2 +
              trajectory_points.at(ref_point_index + 1).w_ref * dis1 ) /
              dis_total;

    double delta_t1, delta_t2;
    delta_t1 = reference_point_.t - trajectory_points.at(ref_point_index).t_ref;
    delta_t2 = trajectory_points.at(ref_point_index + 1).t_ref - reference_point_.t;

    reference_point_.yaw =
            ((trajectory_points.at(ref_point_index).yaw_ref +
            trajectory_points.at(ref_point_index).w_ref * delta_t1) *
            dis2 + (trajectory_points.at(ref_point_index + 1).yaw_ref -
            trajectory_points.at(ref_point_index + 1).w_ref * delta_t2) *
            dis1 ) / dis_total;
    
    reference_point_.x =
            ((trajectory_points.at(ref_point_index).x_ref +
            trajectory_points.at(ref_point_index).v_ref * delta_t1 *
            cos(trajectory_points.at(ref_point_index).yaw_ref * 0.5 +
            reference_point_.yaw * 0.5)) * dis2 +
            (trajectory_points.at(ref_point_index + 1).x_ref -
            trajectory_points.at(ref_point_index + 1).v_ref * delta_t2 *
            cos(trajectory_points.at(ref_point_index + 1).yaw_ref * 0.5 +
            reference_point_.yaw * 0.5)) * dis1 ) / dis_total;
    
    reference_point_.y =
            ((trajectory_points.at(ref_point_index).y_ref +
            trajectory_points.at(ref_point_index).v_ref * delta_t1 *
            sin(trajectory_points.at(ref_point_index).yaw_ref * 0.5 +
            reference_point_.yaw * 0.5)) * dis2 +
            (trajectory_points.at(ref_point_index + 1).y_ref -
            trajectory_points.at(ref_point_index + 1).v_ref * delta_t2 *
            sin(trajectory_points.at(ref_point_index + 1).yaw_ref * 0.5 +
            reference_point_.yaw * 0.5)) * dis1 ) / dis_total;

    p_savedata_->file << "[reference_point_tracking] "
                      << " Time "    << reference_point_.t
                      << " x_ref "   << reference_point_.x
                      << " y_ref "   << reference_point_.y
                      << " yaw_ref " << reference_point_.yaw
                      << " v_ref "   << reference_point_.v
                      << " v_ref "   << reference_point_.w
                      << " t "       << reference_point_.t << endl;
}

