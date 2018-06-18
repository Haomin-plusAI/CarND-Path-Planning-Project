#ifndef PATH_GENERATOR_H
#define PATH_GENERATOR_H
#include <iostream>
#include <vector>
#include "trajectory.h"
#include "vehicle.h"
#include "state_machine.h"

using namespace std;

class PathGenerator
{
    public:

        const Vehicle& ego;
        Trajectory& current_trajectory;

        
        PathGenerator(const Vehicle& ego, Trajectory& trajectory);

        ~PathGenerator();


        // TODO When generating path we need to give the option on whether
        // we want to force a lane centering or simply use the current lane 
        // coordinate d

        /**
         * Generates a number of paths given a set of target conditions
         * 
         */ 
        vector<Trajectory> generatePaths(double target_s, double target_d, 
                                         double target_s_speed, double target_d_speed,
                                         double target_s_acc, double target_d_acc,
                                         double std_s, double std_d, 
                                         int count, int from_point_index, double time_interval);

        vector<Trajectory> generatePaths(const State& state, const Vehicle& ego, 
                                         Trajectory current_trajectory, int from_point_index, int path_count, double time_interval);
    
    private:

        /**
         * @brief Computes the coefficients of the quintic polynomial for a jerk
         * minimized trajectory
         * 
         * @param start the start position, velocity and acceleration
         * @param end the desired end position, velocity and acceleration
         * @param T the time period in seconds
         * @return vector<double> the 6 coefficients vector of the quintic polynomial that minimizes jerk
         */
        vector<double> JMT(vector< double> start, vector <double> end, double T);


        /**
         * @brief Appends a trajectory with the start and end targets in Frenet coordinates
         * to the supplied trajectory
         * 
         * @param start_s the vector of longitudinal start  position , velocity and acceleration
         * @param end_s the vector of longitudinal end position, velocity and acceleration
         * @param start_d the vector of lateral start position, velocity and acceleration
         * @param end_d the vector of lateral end position, velocity and acceleration
         * @param trajectory the trajectory to append to         
         * @param time_interval the time interval in seconds to execute this trajectory
         */
        void appendPath(vector<double> start_s, vector<double> end_s, 
                                vector<double> start_d, vector<double> end_d, 
                                Trajectory &trajectory, double time_interval);
};


#endif