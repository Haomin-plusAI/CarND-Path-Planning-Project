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
        
        /**
         * @brief Predicts the vehicle's future position at time t (in seconds)
         * We assume constant speed and 0 acceleration
         * 
         * @param t the timestep into the future in seconds
         * @return Vehicle the vehicle at timestep t in the future
         */
        Vehicle predictFuturePosition(double t) const;

        /**
         * @brief Checks which vehicles are ahead of the current vehicle on the given lane
         * 
         * @param others the other vehicles which may be ahead
         * @param lane the lane to check against
         * @return vector<Vehicle>  the vehicles ahead of the current vehicle
         */
        vector<Vehicle> ahead(const vector<Vehicle>& others, int lane) const;

        /**
         * @brief Checks which vehicles are behind of the current vehicle on the given lane
         * 
         * @param others the other vehicles which may be behind
         * @param lane the lane to check against
         * @return vector<Vehicle>  the vehicles behind of the current vehicle
         */
        vector<Vehicle> behind(const vector<Vehicle>& others, int lane) const;

        

        double getSpeed() const;
};
#endif