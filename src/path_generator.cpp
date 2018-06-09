#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"
#include "path_generator.h"
#include "vehicle.h"
#include "trajectory.h"
#include "map.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;



PathGenerator::PathGenerator(const Vehicle& ego, Trajectory& current_trajectory)
: ego(ego), current_trajectory(current_trajectory)
{}

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
    
    if(traj_size > 0){
        last_x = first_traj.xs[traj_size - 1];
        last_y = first_traj.ys[traj_size - 1];
        last_s = first_traj.ss[traj_size - 1];
        last_d = first_traj.ds[traj_size - 1];
        last_s_vel = first_traj.s_vels[traj_size - 1];
        last_d_vel = first_traj.d_vels[traj_size - 1];
        last_s_acc = first_traj.s_accs[traj_size - 1];
        last_d_acc = first_traj.d_accs[traj_size - 1];
    }

    

    vector<double> start_s = {last_s, last_s_vel, last_s_acc};
    vector<double> end_s = {target_s, target_s_speed, target_s_acc};
    
    vector<double> start_d = {last_d, last_d_vel, last_d_acc};
    vector<double> end_d = {target_d, target_d_speed, target_d_acc};

    vector<double> coeffs_s = this->JMT(start_s, end_s, time_interval);
    vector<double> coeffs_d = this->JMT(start_d, end_d, time_interval);

    cout << "[P] start_s=" << start_s[0] << " end_s=" << end_s[0] << endl;

    // TODO put in a separate function
    int total_points = time_interval / 0.02;
    int points_remaining = total_points - first_traj.size();
    Map& map = Map::getInstance();
    for(int i = 0; i < points_remaining; ++i)
    {
        // TODO we should store this 0.02 update frequency in some constants
        double t = 0.02 * (i + 1);
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
        // TODO fix the theta angle
        first_traj.add(x_y[0], x_y[1], 
                       s_t, s_t_dot, s_t_dot_dot, 
                       d_t, d_t_dot, d_t_dot_dot, 0.0);

        cout << "[" << i << "] jerk_s=" << s_jerk << " jerk_d=" << d_jerk << endl;   
        // cout << "(Updated) s[" << i << "]: pos= " << s_t << " vel="<< s_t_dot << " acc=" << s_t_dot_dot << endl;                
        // cout << "(Updated) d[" << i << "]: pos= " << d_t << " vel="<< d_t_dot << " acc=" << d_t_dot_dot << endl;                
        // cout << "Car speed MPS=" << car_speed << " mps=" << car_speed_mps << endl;
        
    }

    trajs.push_back(first_traj);

    return trajs;
}



vector<double> PathGenerator::JMT(vector< double> start, vector <double> end, double T)
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































PathGenerator::~PathGenerator(){}
