#include <iostream>
#include <vector>
#include "trajectory.h"
#include "spline.h"
#include "helpers.h"
#include "map.h"
#include <math.h>

using namespace std;

Trajectory::Trajectory() {}

Trajectory::~Trajectory() {}

void Trajectory::add(double x, double y,
                     double s, double s_dot, double s_dot_dot, double s_dot_dot_dot,
                     double d, double d_dot, double d_dot_dot, double d_dot_dot_dot,
                     double yaw)
{
    this->xs.push_back(x);
    this->ys.push_back(y);

    this->ss.push_back(s);
    this->s_vels.push_back(s_dot);
    this->s_accs.push_back(s_dot_dot);
    this->s_jks.push_back(s_dot_dot_dot);

    this->ds.push_back(d);
    this->d_vels.push_back(d_dot);
    this->d_accs.push_back(d_dot_dot);
    this->d_jks.push_back(d_dot_dot_dot);

    this->yaws.push_back(yaw);
}

int Trajectory::size() const
{
    return this->xs.size();
}

double Trajectory::averageSpeed(double point_interval) const
{
    int pts_count = this->size();
    if (pts_count < 2)
    {
        return 0.0;
    }

    double vel_sum = 0.0;
    for (int i = 0; i < pts_count; ++i)
    {
        vel_sum += this->s_vels[i];
    }
    return vel_sum / pts_count;
}

vector<double> Trajectory::averageAcceleration(double point_time_interval) const
{
    int pts_count = this->size();
    if (pts_count < 2)
    {
        return {0.0, 0.0};
    }

    double s_acc_sum = 0.0;
    double d_acc_sum = 0.0;
    for (int i = 0; i < pts_count; ++i)
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
    this->s_jks.erase(this->s_jks.begin(), this->s_jks.begin() + n);

    this->ds.erase(this->ds.begin(), this->ds.begin() + n);
    this->d_vels.erase(this->d_vels.begin(), this->d_vels.begin() + n);
    this->d_accs.erase(this->d_accs.begin(), this->d_accs.begin() + n);
    this->d_jks.erase(this->d_jks.begin(), this->d_jks.begin() + n);

    this->yaws.erase(this->yaws.begin(), this->yaws.begin() + n);
}

Trajectory Trajectory::clone(int up_to_index) const
{
    Trajectory copy = Trajectory();
    for (int i = 0; i < this->size() && i < up_to_index; ++i)
    {
        copy.add(this->xs[i], this->ys[i],
                 this->ss[i], this->s_vels[i], this->s_accs[i], this->s_jks[i],
                 this->ds[i], this->d_vels[i], this->d_accs[i], this->d_jks[i],
                 this->yaws[i]);
    }

    return copy;
}

void Trajectory::smoothen(int from_index)
{
    tk::spline s;
    // s.set_points(this->xs, this->ys);
    vector<double> new_xs;
    vector<double> new_ys;

    new_xs.push_back(this->xs[0]);
    new_ys.push_back(this->ys[0]);
    for (int i = 0 + 1; i < this->xs.size(); ++i)
    {
        double prev_x = new_xs[new_xs.size() - 1];
        double prev_y = new_ys[new_xs.size() - 1];
        double cur_x = this->xs[i];
        double cur_y = this->ys[i];

        double dist = distance(prev_x, prev_y, cur_x, cur_y);
        // cout << "** DISTANCE = " << dist;
        // cout << "** P1 = " << prev_x << ", " << prev_y << endl;
        // cout << "** P2 = " << cur_x << ", " << cur_y << endl;
        if (dist < 0.5)
        {
            continue;
        }
        new_xs.push_back(cur_x);
        new_ys.push_back(this->ys[i]);
    }
    if (new_xs.size() < 3)
    {
        cout << "************************" << endl;
        return;
    }

    s.set_points(new_xs, new_ys);

    // double avg_speed = this->averageSpeed(50);

    // double start_x = new_xs[0];
    // double start_y = new_ys[0];

    // double end_x = new_xs[new_xs.size() - 1];
    // double end_y = s(end_x);

    // double dist = distance(start_x, start_y, end_x, end_y);
    // double angle = atan2(end_y - start_y, end_x - start_x);

    // Now fit 50 points on this line

    // double ref_x = start_x;
    // for(int i = 1; i < this->xs.size(); ++i)
    // {
    //     double prev_x = this->xs[i-1];
    //     double prev_y = this->ys[i-1];

    //     double N = dist / (0.02 * avg_speed);
    //     // double x = start_x + cos(angle) * 0.02 * avg_speed * (i-1);
    //     double x = ref_x + (end_x / N);
    //     double y = s(x);

    //     ref_x = x;

    //     vector<double> frenet = Map::getInstance().toFrenet(x, y, angle);
    //     this->xs[i] = x;
    //     this->ys[i] = y;
    //     this->ss[i] = frenet[0];
    //     this->ds[i] = frenet[1];

    //     // Calculate distance between two points
    //     // Calculate acceleration

    // }

    for (int i = from_index; i < this->xs.size(); ++i)
    {
        double x_prev = this->xs[i - 1];
        double y_prev = s(x_prev);

        double x_now = this->xs[i];
        double y_now = s(x_now);

        double angle = atan2(y_now - y_prev, x_now - x_prev);
        vector<double> frenet = Map::getInstance().toFrenet(x_now, y_now, angle);
        this->xs[i] = x_now;
        this->ys[i] = y_now;
        this->ss[i] = frenet[0];
        this->ds[i] = frenet[1];
    }

    // double avg_speed = this->averageSpeed(50);

    // for(int i = 0; i < this->xs.size() - 1; ++i)
    // {
    //     double cur_x = this->xs[i];
    //     double cur_y = this->ys[i];

    //     double next_x = this->xs[i + 1];
    //     double next_y = this->ys[i + 1];

    //     double vel = distance(cur_x, cur_y, next_x, next_y) / CONTROLLER_UPDATE_RATE_SECONDS;
    //     cout << "** VELOCITY " << vel << " per second" << endl;
    //     if(vel < MAX_SPEED_METERS_PER_SECOND)
    //     {
    //         continue;
    //     }

    //     // We have breached the maximum velocity - therefore we must adjust this point
    //     double theta = atan2(next_y - cur_y, next_x - cur_x);

    //     double new_next_x = cur_x + cos(theta) * 0.015;
    //     double new_next_y = cur_y + sin(theta) * 0.015;

    //     vector<double> frenet = Map::getInstance().toFrenet(new_next_x, new_next_y, theta);
    //     this->xs[i+1] = new_next_x;
    //     this->ys[i+1] = new_next_y;
    //     this->ss[i+1] = frenet[0];
    //     this->ds[i+1] = frenet[1];
    // }
}

bool Trajectory::feasible() const
{
    double avg_speed = this->averageSpeed(50);
    if (avg_speed > MAX_SPEED_METERS_PER_SECOND)
    {
        cout << "*********** AVERAGE SPEED LIMIT BREACHED: " << avg_speed << endl;
        return false;
    }

    return false;
}
