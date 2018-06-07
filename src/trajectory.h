#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <iostream>
#include <vector>

using namespace std;

class Trajectory
{
    public:

        vector<double> xs;
        vector<double> ys;
        
        vector<double> ss;        
        vector<double> s_vels;
        vector<double> s_accs;
        
        vector<double> ds;
        vector<double> d_vels;
        vector<double> d_accs;
        
        vector<double> yaws;

        /**
         * Constructor
         */ 
        Trajectory();
        

        /**
         * Destructor
         */ 
        virtual ~Trajectory();

        void add(double x, double y, 
                 double s, double s_dot, double s_dot_dot,
                 double d, double d_dot, double d_dot_dot,
                 double yaw);
        int size();
        
        /**
         * Calculates the average squared acceleration, also known as the jerk, over all points.
         * 
         * @param point_interval the interval in seconds between two consecutive points
         * @return the average longitudinal speed across all points in the trajectory
         */
        double averageSpeed(double point_interval);


        
        /**
         * Computes the longitudinal and lateral acceleration
         * 
         * @param point_time_interval the interval in seconds between two consecutive points
         * @return a vector contain the average acceleration in longitudinal 
         * and lateral positions across all points
         */ 
        vector<double> averageAcceleration(double point_time_interval);
        
        /**
         * Removes the first n waypoints in the trajectory
         */ 
        void removeFirstN(int n);

        


};



#endif