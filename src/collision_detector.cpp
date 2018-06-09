#include "collision_detector.h"
#include <iostream>
#include <vector>
#include "helpers.h"

using namespace std;


Collision::Collision(const Vehicle& v, const bool willCollide, 
                     const double collision_point_x, const double collision_point_y,
                     const double timestep)
                     :v(v), willCollide(willCollide), 
                      collision_point_x(collision_point_x), collision_point_y(collision_point_y),
                      collision_timestep(collision_timestep)
{
    
}

CollisionDetector::CollisionDetector(){}

CollisionDetector::CollisionDetector(const Trajectory& trajectory)
{
    this->trajectory = trajectory;    
}

Collision CollisionDetector::predictCollision(const Vehicle &vehicle, double timestep)
{    
    for(int i = 0; i < trajectory.size(); ++i)
    {
        double ref_x = trajectory.xs[i];       
        double ref_y = trajectory.ys[i];       

        double v_predcited_x = vehicle.x + vehicle.vx * timestep;
        double v_predcited_y = vehicle.y + vehicle.vy * timestep;

        double dist = distance(ref_x, ref_y, v_predcited_x, v_predcited_y);
        if(dist < 3)
        {
            return Collision(vehicle, true, ref_x, ref_y, (double) i);            
        }
    }
    return Collision(vehicle, false, 0.0, 0.0, 0.0);

}

CollisionDetector::~CollisionDetector() {}