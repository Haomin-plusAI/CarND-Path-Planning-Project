#include <iostream>
#include <math.h>
#include "vehicle.h"
#include "helpers.h"

using namespace std;

Vehicle::Vehicle(){}

Vehicle::Vehicle(int id, double x, double y, double vx, double vy, double s, double d, double t){
    this->id = id;
    this->x = x;
    this->y = y;
    this->vx = vx;
    this->vy = vy;
    this->s = s;
    this->d = d;
    this->t = t;
    
    this->theta = getTheta(vx, vy);    
    this->isInLane = isWithinLane(this->d, 4.0, 1.5);;
    this->lane =  calculateLane(this->d, 4.0, 1.5);
}

Vehicle Vehicle::predictNextPosition(double t1, const vector<double> &maps_x, const vector<double> &maps_y){
    double newX = this->x + this->vx * t1;
    double newY = this->y + this->vy * t1;
    vector<double> frenet = getFrenet(newX, newY, this->theta, maps_x, maps_y);

    // vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)

    return Vehicle(this->id, newX, newY, this->vx, this->vy, frenet[0], frenet[1], t1);
}

double Vehicle::getSpeed()
{
    return sqrt(this->vx * this->vx + this->vy * this->vy);
}

Vehicle::~Vehicle() {}


// Compute angle of travel using https://www.khanacademy.org/science/physics/two-dimensional-motion/two-dimensional-projectile-mot/a/what-are-velocity-components