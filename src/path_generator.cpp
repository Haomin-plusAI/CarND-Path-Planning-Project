#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"
#include "path_generator.h"
#include "vehicle.h"
#include "trajectory.h"
#include "map.h"
#include "state_machine.h"
#include "helpers.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

PathGenerator::PathGenerator(const Vehicle &ego, Trajectory &current_trajectory)
    : ego(ego), current_trajectory(current_trajectory)
{
}

vector<Trajectory> PathGenerator::generatePaths(const State &state, const Vehicle &ego,
                                                Trajectory current_trajectory,
                                                int from_point_index,
                                                int path_count, double time_interval)
{
    double target_s = 0.0;
    double target_s_vel = 0.0;
    double target_s_acc = 0.0;

    double target_d = 0.0;
    double target_d_vel = 0.0;
    double target_d_acc = 0.0;

    double std_s = 5.0;
    double std_d = 0.0;

    double speed_at_index = current_trajectory.s_vels[from_point_index];
    double acc_at_index = current_trajectory.s_accs[from_point_index];

    // cout << "EGO D STATE = " << state.d_state << endl;
    switch (state.s_state)
    {
    case LongitudinalState::MAINTAIN_COURSE:
        target_s_vel = speed_at_index;
        break;
    case LongitudinalState::ACCELERATE:
        // Increase speed by 10%
        target_s_vel = speed_at_index == 0 ? 3.0 : speed_at_index * (1.1);
        break;
    case LongitudinalState::DECELERATE:
        target_s_vel = speed_at_index * 0.80;
        break;
    case LongitudinalState::STOP:
        target_s_vel = speed_at_index * 0.6;
    }

    if (target_s_vel > 21)
    {
        // 22m/s ~ 50 MPH
        target_s_vel = 21;
    }
    // cout << "**** DESIRED SPEED =" << target_s_vel << endl;

    double s_offset = 0.0;
    double d_at_index = current_trajectory.ds[from_point_index];
    switch (state.d_state)
    {
    case LateralState::STAY_IN_LANE:
        // target_d = d_at_index;
        target_d = getLaneCenterFrenet(state.current_lane);
        ;
        break;
    case LateralState::PREPARE_CHANGE_LANE_LEFT:
        // target_d = d_at_index;
        target_d = getLaneCenterFrenet(state.current_lane);
        ;
        // target_d = d_at_index - 4.0;
        break;
    case LateralState::PREPARE_CHANGE_LANE_RIGHT:
        // target_d = d_at_index;
        target_d = getLaneCenterFrenet(state.current_lane);
        ;
        // target_d = d_at_index + 4.0;
        break;
    case LateralState::CHANGE_LANE_LEFT:
        // TODO find center of left lane
        target_d = getLaneCenterFrenet(state.future_lane);
        std_d = 1.0;
        // s_offset = 5.0;
        break;
    case LateralState::CHANGE_LANE_RIGHT:
        target_d = getLaneCenterFrenet(state.future_lane);
        std_d = 1.0;
        // s_offset = 5.0;
        break;
    }

    if (current_trajectory.size() < 5)
    {
        target_d = current_trajectory.ds[0];
    }

    // cout << "******** START S = " << current_trajectory.ss[from_point_index] << endl;
    target_s = (current_trajectory.ss[from_point_index] + target_s_vel * time_interval);
    // target_s = (current_trajectory.ss[from_point_index] + target_s_vel * 1.0);

    return this->generatePaths(target_s, target_d, target_s_vel,
                               0.0, 0.0, 0.0, std_s, std_d,
                               path_count, from_point_index + 1, time_interval);
}

vector<Trajectory> PathGenerator::generatePaths(double target_s, double target_d,
                                                double target_s_speed, double target_d_speed,
                                                double target_s_acc, double target_d_acc,
                                                double std_s, double std_d,
                                                int count, int from_point_index, double time_interval)
{
    vector<Trajectory> trajs;

    Trajectory first_traj = this->current_trajectory.clone(from_point_index);
    int traj_size = first_traj.size();
    double last_x = 0.0;
    double last_y = 0.0;
    double last_s = 0.0;
    double last_d = 0.0;
    double last_s_vel = 0.0;
    double last_d_vel = 0.0;
    double last_s_acc = 0.0;
    double last_d_acc = 0.0;

    if (traj_size > 0)
    {
        last_x = first_traj.xs[traj_size - 1];
        last_y = first_traj.ys[traj_size - 1];
        last_s = first_traj.ss[traj_size - 1];
        last_d = first_traj.ds[traj_size - 1];
        last_s_vel = first_traj.s_vels[traj_size - 1];
        last_d_vel = first_traj.d_vels[traj_size - 1];
        last_s_acc = first_traj.s_accs[traj_size - 1];
        last_d_acc = first_traj.d_accs[traj_size - 1];
    }

    // cout << "** LAST S = " << last_s << endl;
    // cout << "** END S = " << target_s << endl;
    vector<double> start_s = {last_s, last_s_vel, last_s_acc};
    vector<double> end_s = {target_s, target_s_speed, target_s_acc};

    vector<double> start_d = {last_d, last_d_vel, last_d_acc};
    vector<double> end_d = {target_d, target_d_speed, target_d_acc};

    vector<double> coeffs_s = this->JMT(start_s, end_s, time_interval);
    vector<double> coeffs_d = this->JMT(start_d, end_d, time_interval);

    this->appendPath(start_s, end_s, start_d, end_d, first_traj, time_interval);
    trajs.push_back(first_traj);

    // first_traj.feasible();

    // trajs.push_back(first_traj);

    // Create a normal distribution and a time based seed
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> s_distrib(target_s, std_s);
    std::normal_distribution<double> d_distrib(target_d, std_d);

    for (int i = 1; i < count; ++i)
    {
        Trajectory traj = this->current_trajectory.clone(from_point_index);
        double new_target_s = s_distrib(generator);
        double new_target_d = d_distrib(generator);

        start_s = {last_s, last_s_vel, last_s_acc};
        end_s = {new_target_s, target_s_speed, target_s_acc};

        start_d = {last_d, last_d_vel, last_d_acc};
        end_d = {new_target_d, target_d_speed, target_d_acc};

        coeffs_s = this->JMT(start_s, end_s, time_interval);
        coeffs_d = this->JMT(start_d, end_d, time_interval);

        this->appendPath(start_s, end_s, start_d, end_d, traj, time_interval);

        trajs.push_back(traj);
    }

    return trajs;
}

void PathGenerator::appendPath(vector<double> start_s, vector<double> end_s,
                               vector<double> start_d, vector<double> end_d,
                               Trajectory &trajectory, double time_interval)
{
    vector<double> coeffs_s = this->JMT(start_s, end_s, time_interval);
    vector<double> coeffs_d = this->JMT(start_d, end_d, time_interval);

    int total_points = time_interval / CONTROLLER_UPDATE_RATE_SECONDS;
    // cout << "^^^^^^ TOTAL POINTS " << total_points << endl;
    int points_remaining = total_points - trajectory.size();
    Map &map = Map::getInstance();

    double last_x = trajectory.xs[trajectory.size() - 1];
    double last_y = trajectory.ys[trajectory.size() - 1];

    double last_s = trajectory.ss[trajectory.size() - 1];
    double last_d = trajectory.ds[trajectory.size() - 1];

    for (int i = 0; i < points_remaining; ++i)
    {
        double t = CONTROLLER_UPDATE_RATE_SECONDS * (i + 1);
        double t_2 = pow(t, 2);
        double t_3 = pow(t, 3);
        double t_4 = pow(t, 4);
        double t_5 = pow(t, 5);

        double s_t = start_s[0] + start_s[1] * t + 0.5 * start_s[2] * t_2 + coeffs_s[3] * t_3 + coeffs_s[4] * t_4 + coeffs_s[5] * t_5;
        double s_t_dot = start_s[1] + start_s[2] * t + 3 * coeffs_s[3] * t_2 + 4 * coeffs_s[4] * t_3 + 5 * coeffs_s[5] * t_4;
        double s_t_dot_dot = start_s[2] + 6 * coeffs_s[3] * t + 12 * coeffs_s[4] * t_2 + 20 * coeffs_s[5] * t_3;
        double s_jerk = 6 * coeffs_s[3] + 24 * coeffs_s[4] * t + 60 * coeffs_s[5] * t_2;

        double d_t = start_d[0] + start_d[1] * t + start_d[2] * 0.5 * t_2 + coeffs_d[3] * t_3 + coeffs_d[4] * t_4 + coeffs_d[5] * t_5;
        double d_t_dot = start_d[1] + start_d[2] * t + 3 * coeffs_d[3] * t_2 + 4 * coeffs_d[4] * t_3 + 5 * coeffs_d[5] * t_4;
        double d_t_dot_dot = start_d[2] + 6 * coeffs_d[3] * t + 12 * coeffs_d[4] * t_2 + 20 * coeffs_d[5] * t_3;
        double d_jerk = 6 * coeffs_d[3] + 24 * coeffs_d[4] * t + 60 * coeffs_d[5] * t_2;

        vector<double> x_y = map.toRealWorldXY(s_t, d_t);
        double x = x_y[0];
        double y = x_y[1];

        double theta = atan2(y - last_y, x - last_x);
        // TODO fix the theta angle
        trajectory.add(x_y[0], x_y[1],
                       s_t, s_t_dot, s_t_dot_dot, s_jerk,
                       d_t, d_t_dot, d_t_dot_dot, d_jerk,
                       theta);

        double dist = distance(last_x, last_y, x, y);
        last_x = x;
        last_y = y;

        // cout << "[" << i << "] jerk_s=" << s_jerk << " jerk_d=" << d_jerk << endl;
        // cout << "(Updated) s[" << i << "]: pos= " << s_t << " vel=" << s_t_dot << " acc=" << s_t_dot_dot << endl;
        // cout << "(Updated) d[" << i << "]: pos= " << d_t << " vel="<< d_t_dot << " acc=" << d_t_dot_dot << endl;
        // cout << "(Updated) x[" << i << "]=" << x_y[0] << " y[" << i << "]=" << x_y[1] << " - dist=" << dist << endl;
        // cout << "Car speed MPS=" << car_speed << " mps=" << car_speed_mps << endl;
    }
}

vector<double> PathGenerator::JMT(vector<double> start, vector<double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */

    // create T matrix
    MatrixXd t_matrix(3, 3);
    t_matrix << pow(T, 3), pow(T, 4), pow(T, 5),
        3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4),
        6 * T, 12 * pow(T, 2), 20 * pow(T, 3);

    // create sf diff vector
    VectorXd sf_diff(3);
    sf_diff << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * pow(T, 2)),
        end[1] - (start[1] + start[2] * T),
        end[2] - start[2];

    VectorXd coeffs = t_matrix.inverse() * sf_diff;

    vector<double> final_coeffs = {start[0], start[1], 0.5 * start[2], coeffs[0], coeffs[1], coeffs[2]};
    return final_coeffs;

    // return {1,2,3,4,5,6};
}

PathGenerator::~PathGenerator() {}
