#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <vector>

using namespace std;

class Vehicle
{
    public:
        int id;
        
        double x;
        double y;
        
        double vx;
        double vy;
        
        double s;
        double d;

        // Timestep
        double t;

        // We can compute those
        int lane;
        bool isInLane;
        double theta;

        /**
         * Constructors
         */
        Vehicle(); 
        Vehicle(int id, double x, double y, double vx, double vy, double s, double d, double t);

        /**
        * Destructor
        */
        virtual ~Vehicle();

        // Returns a new vehicle at the next timestep
        Vehicle predictNextPosition(double t1, const vector<double> &maps_x, const vector<double> &maps_y);

        double getSpeed();
};
#endif