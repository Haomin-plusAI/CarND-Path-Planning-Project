#include "collision_detector.h"
#include <iostream>
#include <vector>
#include "helpers.h"
#include "map.h"

using namespace std;

Collision::Collision(const Vehicle &v, const bool willCollide,
                     const double collision_point_x, const double collision_point_y,
                     const double collision_timestep)
    : v(v), willCollide(willCollide),
      collision_point_x(collision_point_x), collision_point_y(collision_point_y),
      collision_timestep(collision_timestep)
{
}

CollisionDetector::CollisionDetector() {}

CollisionDetector::CollisionDetector(const Trajectory &trajectory)
{
    this->trajectory = trajectory;
}

Collision CollisionDetector::predictCollision(const Vehicle &vehicle, double timestep)
{
    Map &map = Map::getInstance();

    // TODO we should store vehicle predicted coordinates somewhere so that it is reused
    for (int i = 0; i < trajectory.size(); ++i)
    {
        double ref_x = trajectory.xs[i];
        double ref_y = trajectory.ys[i];

        double v_predcited_x = vehicle.x + vehicle.vx * timestep * i;
        double v_predcited_y = vehicle.y + vehicle.vy * timestep * i;

        vector<double> frenet = map.toFrenet(v_predcited_x, v_predcited_y, vehicle.theta);

        // TODO check lane here
        int ego_lane = calculateLane(this->trajectory.ds[i], DEFAULT_LANE_SPACING, DEFAULT_LANE_INSIDE_OFFSET);
        int v_lane = calculateLane(frenet[1], DEFAULT_LANE_SPACING, DEFAULT_LANE_INSIDE_OFFSET);

        // if (ego_lane < 0 || ego_lane >= LANES_COUNT)
        // {
        //     continue;
        // }

        // if (v_lane < 0 || v_lane >= LANES_COUNT)
        // {
        //     continue;
        // }

        double dist = distance(ref_x, ref_y, v_predcited_x, v_predcited_y);

        // TODO Put this into a constant
        if (dist < 15)
        {
            cout << ">>>>>>>>>>>>** Collision timestep = " << i << endl;
            return Collision(vehicle, true, ref_x, ref_y, (double)i);
        }
    }
    return Collision(vehicle, false, 0.0, 0.0, 0.0);
}

CollisionDetector::~CollisionDetector() {}

Collision::~Collision() {}
