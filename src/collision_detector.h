#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H
#include <iostream>
#include <vector>
#include "vehicle.h"
#include "trajectory.h"

using namespace std;


class Collision
{
    public:
        const Vehicle& v;
        const bool willCollide;
        const double collision_point_x;
        const double collision_point_y;
        const double collision_timestep;
    
        Collision(const Vehicle& v, const bool willCollide, 
                  const double collision_point_x, const double collision_point_y, const double collision_timestep);
        ~Collision();
};

class CollisionDetector
{
    public:
        Trajectory trajectory;

        /**
         * Constructor
         */
        CollisionDetector();
        CollisionDetector(const Trajectory& trajectory);

        /**
         * Destructor
         */
        virtual ~CollisionDetector();

        /**
         * Computes the coordinates (x, y) and time of a potential collision with another vehicle.
         * We assume constant speed and zero acceleration for the passed vehicle
         * @param vehicle the vehicle we want to check whether we will collide with
         * @param timestep the timestep unit (e.g. 0.5s, 1s, etc.) between successive points
         * 
         */ 
        Collision predictCollision(const Vehicle &vehicle, double timestep);
};


#endif