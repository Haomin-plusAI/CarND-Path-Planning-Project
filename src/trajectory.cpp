#include <iostream>
#include <vector>
#include "trajectory.h"

using namespace std;

Trajectory::Trajectory(){}

Trajectory::~Trajectory(){}

void Trajectory::add(double x, double y, double s, double d, double yaw)
{
    this->xs.push_back(x);
    this->ys.push_back(y);
    this->ss.push_back(s);
    this->ds.push_back(d);
    this->yaws.push_back(yaw);
}

int Trajectory::size()
{
    return this->xs.size();
}

double Trajectory::averageSpeed(double point_interval)
{
    int pts_count = this->size();
    if(pts_count < 2)
    {
        return 0.0;
    }
    double diff = this->ss[pts_count - 1] - this->ss[0];
    return diff / (pts_count * point_interval);
}

vector<double> Trajectory::averageAcceleration(double point_time_interval)
{
    int pts_count = this->size();
    if(pts_count < 2)
    {
        return {0.0, 0.0};
    }

    double a_lon = 0.0;
    double a_lat = 0.0;
    for(int i = 1; i < pts_count; ++i)
    {
        a_lon += (this->ss[i] - this->ss[i-1]);
        a_lat += (this->ds[i] - this->ds[i-1]);
    }

    double divider = pts_count - 1;
    double multiplier = divider * point_time_interval;
    return { (a_lon / divider) / point_time_interval, (a_lat /  divider) / point_time_interval };
}


void Trajectory::removeFirstN(int n)
{
    // v.erase( v.begin(), v.size() > N ?  v.begin() + N : v.end() );
     this->xs.erase(this->xs.begin(), this->xs.begin() + n);   
     this->ys.erase(this->ys.begin(), this->ys.begin() + n);   
     this->ss.erase(this->ss.begin(), this->ss.begin() + n);   
     this->ds.erase(this->ds.begin(), this->ds.begin() + n);   
     this->yaws.erase(this->yaws.begin(), this->yaws.begin() + n);   
}