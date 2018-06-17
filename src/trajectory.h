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
        vector<double> s_jks;
                        
        vector<double> ds;
        vector<double> d_vels;
        vector<double> d_accs;
        vector<double> d_jks;

        
        vector<double> yaws;

        /**
         * Constructor
         */ 
        Trajectory();
        

        /**
         * Destructor
         */ 
        virtual ~Trajectory();
        

        /**
         * Add a new point to the trajectory
         */ 
        void add(double x, double y, 
                 double s, double s_dot, double s_dot_dot, double s_dot_dot_dot,
                 double d, double d_dot, double d_dot_dot, double d_dot_dot_dot,
                 double yaw);
        
        /**
         * @return the number of points in the trajectory
         */ 
        int size() const;
        
        /**
         * Calculates the average squared acceleration, also known as the jerk, over all points.
         * 
         * @param point_interval the interval in seconds between two consecutive points
         * @return the average longitudinal speed across all points in the trajectory
         */
        double averageSpeed(double point_interval) const; 


        // vector<double> 

        
        /**
         * Computes the longitudinal and lateral acceleration
         * 
         * @param point_time_interval the interval in seconds between two consecutive points
         * @return a vector contain the average acceleration in longitudinal 
         * and lateral positions across all points
         */ 
        vector<double> averageAcceleration(double point_time_interval) const;
        
        /**
         * Removes the first n waypoints in the trajectory
         */ 
        void removeFirstN(int n);


        /**
         * Clones the underlying trajectory from points [0, up_to_index)
         * @param up_to_index
         * @return a new trajectory that contains all points up to the specified index excluded
         */ 
        Trajectory clone(int up_to_index) const;

        
        /**
         * @brief Attempts to smoother the trajectory using a polynomial fit
         * 
         */
        void smoothen(int from_index);

        /**
         * @brief Checks whether the path is feasible
         * 
         * @return true 
         * @return false 
         */
        bool feasible() const; 


        

            

        


};



#endif