#include <iostream>
#include "path_validator.h"
#include "vehicle.h"
#include "state_machine.h"
#include "collision_detector.h"
#include "trajectory.h"
#include "helpers.h"

using namespace std;

PathValidator::PathValidator()
{}

PathValidator::~PathValidator()
{}

PathValidationStatus PathValidator::validate(const Vehicle& ego,
                                             vector<Vehicle> others,
                                             const State& state,
                                             const Trajectory& trajectory,
                                             int from_point) const
{
    // if(!isLaneValid(state.current_lane) || !isLaneValid(state.future_lane))
    // {
    //     return PathValidationStatus::OUTSIDE_OF_LANE;
    // }


    double total_s_acc = 0.0;
    double total_d_acc = 0.0;
    double total_s_vel = 0.0;
    double total_d_vel = 0.0;
    double total_s_jerk = 0.0;
    double total_d_jerk = 0.0;
    double prev_s_vel = 0.0;
    double prev_d_vel = 0.0;
    
    for(int i = from_point; i < trajectory.xs.size(); ++i)
    {
        double last_s = trajectory.ss[i - 1];
        double last_d = trajectory.ds[i - 1];
        
        double cur_s = trajectory.ss[i];
        double cur_d = trajectory.ds[i];

        double s_vel = cur_s - last_s;
        double d_vel = cur_d - last_d;

        total_s_vel += s_vel;
        total_d_vel += d_vel;
        
        double s_acc = i < from_point + 1 ? 0.0 : s_vel - prev_s_vel;
        double d_acc = i < from_point + 1 ? 0.0 : d_vel - prev_d_vel;

        total_s_acc += abs(s_acc);
        total_d_acc += abs(d_acc);

        prev_s_vel = s_vel;
        prev_d_vel = d_vel;

        // TODO Check turning angle should be less than a specified value
    }

    cout << "------------- TOTAL S_ACC = " << total_s_acc << endl;
    cout << "------------- TOTAL D_ACC = " << total_d_acc << endl;

    double total_acc = 0;
    double prev_velocity = 0.0;
    double total_velocity = 0.0;
    int segment_size = trajectory.size() - from_point;
    for(int i = from_point;i < trajectory.size(); ++i)
    {
        double last_x = trajectory.xs[i - 1];
        double last_y = trajectory.ys[i - 1];
        
        double cur_x = trajectory.xs[i];
        double cur_y = trajectory.ys[i];

        double vel = distance(last_x, last_y, cur_x, cur_y);     
        // cout << "**** DISTANCE  = " << vel << endl;           
        // cout << "**** VELOCITY  = " << vel / CONTROLLER_UPDATE_RATE_SECONDS << endl;           
        
        if(vel / CONTROLLER_UPDATE_RATE_SECONDS > 22.2)
        {
            cout << "**** MAXIMUM VELOCITY ABOVE THRESHOLD = " << vel / CONTROLLER_UPDATE_RATE_SECONDS << endl;
            return PathValidationStatus::VELOCITY_ABOVE_THRESHOLD;
        }


        total_velocity += vel;
        double acc = i == from_point ? 0.0 : vel - prev_velocity;        
        total_acc += acc;        

        prev_velocity = vel;        
    }
    double avg_velocity = total_velocity  / (segment_size * CONTROLLER_UPDATE_RATE_SECONDS);

    // cout << "**** AVG VELOCITY = " << avg_velocity << endl;
    // cout << "**** TOTAL ACCELERATION = " << total_acc << endl;
    
    if(abs(avg_velocity) < 3)
    {
        cout << "**** AVG VELOCITY BELOW THRESHOLD = " << avg_velocity << endl;
        return PathValidationStatus::AVERAGE_SPEED_BELOW_THRESHOLD;
    }
    
    if(abs(total_acc) >= 10.0)
    {
        cout << "**** TOTAL ACCELERATION EXCEEDED " << total_acc << endl;
        return PathValidationStatus::TOTAL_ACCELERATION_ABOVE_THRESHOLD;
    }

    



    // // TODO maybe have a closest vehicles(ego, vehicles, lane) function that
    // // returns sorted vector of closest vehicles to ego on lane
    // double min_distance = VEHICLE_DISTANCE_THRESHOLD_METERS;
    // Vehicle closest_vehicle;
    // for(const Vehicle& v : others)
    // {
    //     // Other car must be ahead in the same current or future lane
    //     if(v.isInLane && (v.lane == state.current_lane || v.lane == state.future_lane) 
    //        && v.s > ego.s)
    //     {
    //         double dist = distance(ego.x, ego.y, v.x, v.y);
    //         if(dist < min_distance)
    //         {
    //             min_distance = dist;
    //             closest_vehicle = v;
    //         }
    //     }
    // }

    // if(min_distance < VEHICLE_DISTANCE_THRESHOLD_METERS)
    // {
    //     CollisionDetector detector = CollisionDetector(trajectory);
    //     Collision collision = detector.predictCollision(closest_vehicle, CONTROLLER_UPDATE_RATE_SECONDS);
    //     if(collision.willCollide)
    //     {
    //         cout << "**************************** COLLISION AT TIMESTEP " << collision.collision_timestep << endl;
    //         // TODO FIX THIS as it could be adjacent vehicle too
    //         return PathValidationStatus::COLLISION_VEHICLE_AHEAD;
    //     }
    // }
    
    
    return PathValidationStatus::VALID;

}