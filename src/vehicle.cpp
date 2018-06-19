#include <iostream>
#include <math.h>
#include "vehicle.h"
#include "helpers.h"
#include "map.h"

using namespace std;

Vehicle::Vehicle() {}

Vehicle::Vehicle(int id, double x, double y, double vx, double vy, double s, double d, double t)
{
    this->id = id;
    this->x = x;
    this->y = y;
    this->vx = vx;
    this->vy = vy;
    this->s = s;
    this->d = d;
    this->t = t;

    this->theta = getTheta(vx, vy);
    this->isInLane = isWithinLane(this->d, 4.0, 1.5);
    this->lane = calculateLane(this->d, 4.0, 1.5);
}

Vehicle Vehicle::predictNextPosition(double t1, const vector<double> &maps_x, const vector<double> &maps_y)
{
    double newX = this->x + this->vx * t1;
    double newY = this->y + this->vy * t1;
    double theta = atan2(newY - this->y, newX - this->x);
    vector<double> frenet = Map::getInstance().toFrenet(newX, newY, theta);

    return Vehicle(this->id, newX, newY, this->vx, this->vy, frenet[0], frenet[1], t1);
}

Vehicle Vehicle::predictFuturePosition(double t) const
{
    double newX = this->x + this->vx * t;
    double newY = this->y + this->vy * t;

    Map &map = Map::getInstance();
    vector<double> frenet = map.toFrenet(newX, newY, this->theta);
    return Vehicle(this->id, newX, newY, this->vx, this->vy, frenet[0], frenet[1], t);
}

double Vehicle::getSpeed() const
{
    return sqrt(this->vx * this->vx + this->vy * this->vy);
}

vector<Vehicle> Vehicle::ahead(const vector<Vehicle> &others, int lane) const
{
    vector<Vehicle> v_ahead;
    for (const Vehicle &v : others)
    {
        if (v.lane != lane)
        {
            continue;
        }
        if (v.s >= this->s)
        {
            v_ahead.push_back(v);
        }
    }

    return v_ahead;
}

vector<Vehicle> Vehicle::behind(const vector<Vehicle> &others, int lane) const
{
    vector<Vehicle> v_behind;
    for (const Vehicle &v : others)
    {
        if (v.lane != lane)
        {
            continue;
        }
        if (v.s < this->s)
        {
            v_behind.push_back(v);
        }
    }

    return v_behind;
}

Vehicle::~Vehicle() {}

// Compute angle of travel using https://www.khanacademy.org/science/physics/two-dimensional-motion/two-dimensional-projectile-mot/a/what-are-velocity-components
