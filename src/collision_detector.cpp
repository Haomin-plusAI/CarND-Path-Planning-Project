#include "collision_detector.h"
#include <iostream>
#include <vector>
#include "helpers.h"

using namespace std;

CollisionDetector::CollisionDetector(){}

CollisionDetector::CollisionDetector(const Trajectory& trajectory)
{
    this->trajectory = trajectory;    
}

vector<double> CollisionDetector::predictCollision(const Vehicle &vehicle, double timestep)
{
    vector<double> collision;
    for(int i = 0; i < trajectory.size(); ++i)
    {
        double ref_x = trajectory.xs[i];       
        double ref_y = trajectory.ys[i];       

        double v_predcited_x = vehicle.x + vehicle.vx * timestep;
        double v_predcited_y = vehicle.y + vehicle.vy * timestep;

        double dist = distance(ref_x, ref_y, v_predcited_x, v_predcited_y);
        if(dist < 1)
        {
            collision.push_back(ref_x);
            collision.push_back(ref_y);
            collision.push_back((double)i);
            cout << "** Predicted collision with vehicle " << vehicle.id
                 << " dist=" << dist << " x=" << ref_x << " y=" << ref_y
                 << " timestep=" << i << endl;
            break;
        }
    }
    return collision;

}

CollisionDetector::~CollisionDetector() {}