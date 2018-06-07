#include <iostream>
#include <vector>
#include "trajectory.h"
#include <math.h>

using namespace std;

Trajectory::Trajectory(){}

Trajectory::~Trajectory(){}

void Trajectory::add(double x, double y, 
                    double s, double s_dot, double s_dot_dot,
                    double d, double d_dot, double d_dot_dot,
                    double yaw)
{
    this->xs.push_back(x);
    this->ys.push_back(y);
    
    this->ss.push_back(s);
    this->s_vels.push_back(s_dot);
    this->s_accs.push_back(s_dot_dot);
    
    this->ds.push_back(d);
    this->d_vels.push_back(d_dot);
    this->d_accs.push_back(d_dot_dot);
    
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

    double vel_sum = 0.0;
    for(int i = 0; i < pts_count; ++i)
    {
        vel_sum += this->s_vels[i];
    }
    return vel_sum / pts_count;    
}

vector<double> Trajectory::averageAcceleration(double point_time_interval)
{
    int pts_count = this->size();
    if(pts_count < 2)
    {
        return {0.0, 0.0};
    }

    double s_acc_sum = 0.0;
    double d_acc_sum = 0.0;
    for(int i = 0; i < pts_count; ++i)
    {
        // TODO should either square it or use abs as we have negative acceleration
        s_acc_sum += this->s_accs[i] * 50.0;
        d_acc_sum += this->d_accs[i] * 50.0;
    }
    return {s_acc_sum, d_acc_sum};
    
    // int pts_count = this->size();
    // if(pts_count < 2)
    // {
    //     return {0.0, 0.0};
    // }

    // double a_lon = 0.0;
    // double a_lat = 0.0;
    // for(int i = 1; i < pts_count; ++i)
    // {
    //     a_lon += (this->ss[i] - this->ss[i-1]);
    //     a_lat += (this->ds[i] - this->ds[i-1]);
    // }

    // double divider = pts_count - 1;
    // double multiplier = divider * point_time_interval;
    // return { (a_lon / divider) / point_time_interval, (a_lat /  divider) / point_time_interval };
}


void Trajectory::removeFirstN(int n)
{
    // v.erase( v.begin(), v.size() > N ?  v.begin() + N : v.end() );
     this->xs.erase(this->xs.begin(), this->xs.begin() + n);   
     this->ys.erase(this->ys.begin(), this->ys.begin() + n);   
     
     this->ss.erase(this->ss.begin(), this->ss.begin() + n);   
     this->s_vels.erase(this->s_vels.begin(), this->s_vels.begin() + n);   
     this->s_accs.erase(this->s_accs.begin(), this->s_accs.begin() + n);   
     
     this->ds.erase(this->ds.begin(), this->ds.begin() + n);   
     this->d_vels.erase(this->d_vels.begin(), this->d_vels.begin() + n);   
     this->d_accs.erase(this->d_accs.begin(), this->d_accs.begin() + n);   
     
     this->yaws.erase(this->yaws.begin(), this->yaws.begin() + n);   
}