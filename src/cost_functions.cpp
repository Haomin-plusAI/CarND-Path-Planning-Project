#include <iostream>
#include <vector>
#include <functional>
#include <math.h>
#include "cost_functions.h"
#include "vehicle.h"
#include "trajectory.h"
#include "state_machine.h"
#include "collision_detector.h"
#include "helpers.h"


double speedCostFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                         const State& state, const double& weight)
{
    double avg_speed = trajectory.averageSpeed(CONTROLLER_UPDATE_RATE_SECONDS);
    double epislon = 0.001;
    if(avg_speed > MAX_SPEED_METERS_PER_SECOND + epislon)
    {
        return weight;
    } 

    double diff = (MAX_SPEED_METERS_PER_SECOND - avg_speed) / MAX_SPEED_METERS_PER_SECOND;
    return weight * (1 - exp(- abs(diff)));
}

double centerOfLaneDistCostFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                                    const State& state, const double& weight)
{
    double final_d = trajectory.ds[trajectory.ds.size() - 1];    
    int lane = calculateLane(final_d, DEFAULT_LANE_SPACING, DEFAULT_LANE_INSIDE_OFFSET);
    int lane_center = getLaneCenterFrenet(lane);
    
    double diff = lane_center - final_d;
    
    return weight * (1 - exp(- abs(diff)));
}


double laneChangeCostFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                              const State& state, const double& weight)
{
    if(state.current_lane == state.future_lane)
    {
        // No penalty if staying on the same lane
        return 0.0;
    }

    // Weight penalty if switching lane
    return weight;        
}


/**
 * @brief Cost function that increases the penalty the closer the car ahead is from the
 * ego vehicle
 * 
 * @param ego 
 * @param others 
 * @param trajectory 
 * @param state 
 * @param weight 
 * @return double 
 */
double distanceToClosestCarAheadCostFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                                             const State& state, const double& weight)
{
    // We are switching lanes and this is not a cancellable operation    
    if(state.d_state == LateralState::CHANGE_LANE_LEFT 
      || state.d_state == LateralState::CHANGE_LANE_RIGHT)
    {
        return 0;
    }

    // Find closest car ahead and get distance
    if(!ego.isInLane)
    {
        return weight;
    }

    double min_distance = VEHICLE_DISTANCE_THRESHOLD_METERS;
    for(const Vehicle& v : others)
    {
        // Other car must be ahead in the same lane
        if(v.isInLane && v.lane == state.future_lane && v.s >= ego.s)
        {
            double dist = distance(ego.x, ego.y, v.x, v.y);
            if(dist < min_distance)
            {
                min_distance = dist;
            }
        }
    }

    double diff = (VEHICLE_DISTANCE_THRESHOLD_METERS - min_distance);
    return weight * (1 - exp(- abs(diff)));
}


double longitudinalDistanceToClosestAdjacentCarFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                                                        const State& state, const double& weight)
{
    // This cost function only applies to lane changes
    if(state.current_lane == state.future_lane && state.current_lane == ego.lane)
    {
        return 0.0;
    }

    double min_distance = VEHICLE_DISTANCE_THRESHOLD_METERS;
    for(const Vehicle& v : others)
    {
        // Other car must be ahead in the same lane
        if(v.isInLane && v.lane == state.future_lane)
        {   
            // Other car is on different lane 
            double dist = abs(ego.s - v.s);                        
            if(dist < min_distance)
            {
                min_distance = dist;
            }
        }
    }
    
    // cout << "**************** DISTANCE ADJACENT LANE = " << min_distance << endl;

    double diff = (VEHICLE_DISTANCE_THRESHOLD_METERS - min_distance);
    return weight * (1 - exp(- abs(diff)));
}





double distanceToClosestCarAheadFutureLaneCostFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                                                       const State& state, const double& weight)
{
    
    if(state.current_lane == state.future_lane)
    {
        return 0.0;
        // return distanceToClosestCarAheadCostFunction(ego, others, trajectory, state, weight);
    }

    // Find closest car ahead and get distance
    if(!ego.isInLane)
    {
        return weight;
    }

    double min_distance = VEHICLE_DISTANCE_THRESHOLD_METERS;
    for(const Vehicle& v : others)
    {
        // Other car must be ahead in the same lane
        if(v.isInLane && v.lane == state.future_lane && v.s > ego.s)
        {
            double dist = distance(ego.x, ego.y, v.x, v.y);
            if(dist < min_distance)
            {
                min_distance = dist;
            }
        }
    }

    double diff = (VEHICLE_DISTANCE_THRESHOLD_METERS - min_distance);
    return weight * (1 - exp(- abs(diff)));
}


/**
 * @brief Computes a cost function that penalises the future lane depending
 * on the average speed of all vehicles ahead
 * 
 * @param ego 
 * @param others 
 * @param trajectory 
 * @param state 
 * @param weight 
 * @return double 
 */
double averageLaneSpeedDiffCostFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                                        const State& state, const double& weight)
{

    // We are switching lanes and this is not a cancellable operation    
    if(state.d_state == LateralState::CHANGE_LANE_LEFT 
      || state.d_state == LateralState::CHANGE_LANE_RIGHT)
    {
        return 0;
    }
    
    // Not in lane so doesn't count
    if(!ego.isInLane)
    {
        return 0.0;
    }

    // Find all vehicles ahead
    vector<Vehicle> ahead = ego.ahead(others, state.future_lane);
    if(ahead.size() == 0)
    {
        return 0.0;
    }

    double speed_avg = 0.0;
    for(const Vehicle& v: ahead)
    {
        speed_avg += v.getSpeed();
    }
    speed_avg /= ahead.size();

    // We penalize the slowest lanes
    double diff = MAX_SPEED_METERS_PER_SECOND - speed_avg;
    return weight * (1 - exp(- abs(diff)));
}



double speedDifferenceWithClosestCarAheadCostFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                                                      const State& state, const double& weight)
{
    // TODO need to review this
    if(!ego.isInLane)
    {
        return weight;
    }

    double min_distance = VEHICLE_DISTANCE_THRESHOLD_METERS;
    Vehicle closest_vehicle;
    for(const Vehicle& v : others)
    {
        // Other car must be ahead in the same lane
        if(v.isInLane && v.lane == state.future_lane && v.s > ego.s)
        {
            double dist = distance(ego.x, ego.y, v.x, v.y);
            if(dist < min_distance)
            {
                min_distance = dist;
                closest_vehicle = v;
            }
        }
    }
    
    if(min_distance >= VEHICLE_DISTANCE_THRESHOLD_METERS)
    {   
        // No need to penalize if vehicle ahead is far enough...
        return 0.0;
    }

    double ego_speed = trajectory.averageSpeed(1.0);
    double v_speed = closest_vehicle.getSpeed();

    // cout << "** Future ego speed (future lane=" << state.future_lane << ", current lane=" << state.current_lane << ") : " << ego_speed 
    //      << " other vehicle (lane=" << closest_vehicle.lane <<  ") : " << closest_vehicle.getSpeed() << endl;

    double diff = v_speed - ego_speed;
    return weight * (1 - exp(- abs(diff)));
}


// TODO Compute the average speed of lanes
double lanesAverageForwardSpeedCarsAhead(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                                                      const State& state, const double& weight)
{
    // TODO need to review this
    if(!ego.isInLane)
    {
        return weight;
    }

    double min_distance = VEHICLE_DISTANCE_THRESHOLD_METERS;
    Vehicle closest_vehicle;
    for(const Vehicle& v : others)
    {
        // Other car must be ahead in the same lane
        if(v.isInLane && v.lane == state.future_lane && v.s > ego.s)
        {
            double dist = distance(ego.x, ego.y, v.x, v.y);
            if(dist < min_distance)
            {
                min_distance = dist;
                closest_vehicle = v;
            }
        }
    }
    
    if(min_distance >= VEHICLE_DISTANCE_THRESHOLD_METERS)
    {   
        // No need to penalize if vehicle ahead is far enough...
        return 0.0;
    }

    double ego_speed = trajectory.averageSpeed(1.0);
    double v_speed = closest_vehicle.getSpeed();

    // cout << "** Future ego speed (future lane=" << state.future_lane << ", current lane=" << state.current_lane << ") : " << ego_speed 
    //      << " other vehicle (lane=" << closest_vehicle.lane <<  ") : " << closest_vehicle.getSpeed() << endl;

    double diff = v_speed - ego_speed;
    return weight * (1 - exp(- abs(diff)));
}





double collisionTimeCostFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                                 const State& state, const double& weight)
{
    // TODO need to review this
    if(!ego.isInLane)
    {
        return weight;
    }

    double min_distance = VEHICLE_DISTANCE_THRESHOLD_METERS;
    Vehicle closest_vehicle;
    for(const Vehicle& v : others)
    {
        // Other car must be ahead in the same lane
        if(v.isInLane && (v.lane == state.current_lane || v.lane == state.future_lane) 
           && v.s > ego.s)
        {
            double dist = distance(ego.x, ego.y, v.x, v.y);
            if(dist < min_distance)
            {
                min_distance = dist;
                closest_vehicle = v;
            }
        }
    }

    
    if(min_distance >= VEHICLE_DISTANCE_THRESHOLD_METERS)
    {   
        // No need to penalize if vehicle ahead is far enough...
        return 0.0;
    }

    CollisionDetector detector = CollisionDetector(trajectory);
    Collision collision = detector.predictCollision(closest_vehicle, CONTROLLER_UPDATE_RATE_SECONDS);
    if(!collision.willCollide)
    {
        // cout << "**** NO COLLISION " << endl;
        // If no collision foreseen then don't penalize
        return 0.0;
    }

    double ego_speed = trajectory.averageSpeed(1.0);
    // cout << "** Collision with vehicle at timestep = " << collision.collision_timestep << endl;

    
    // Otherwise penalize as a factor of the time to collision - the further away in time the better
    return weight * (1 - exp(-collision.collision_timestep));
}








// double futureDistanceToClosestCarAheadCostFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
//                                              const State& state, const double& weight)
// {
//     // Find closest car ahead and get distance
//     if(!ego.isInLane)
//     {
//         return weight;
//     }

//     int traj_size = trajectory.size();
//     double timesteps_ahead = traj_size * CONTROLLER_UPDATE_RATE_SECONDS;
//     double ego_future_s = trajectory.ss[traj_size - 1];
//     double ego_future_x = trajectory.xs[traj_size - 1];
//     double ego_future_y = trajectory.ys[traj_size - 1];


//     double min_distance = VEHICLE_DISTANCE_THRESHOLD_METERS;
//     for(const Vehicle& v : others)
//     {
//         Vehicle v_future = v.predictFuturePosition(timesteps_ahead);
        
        
//         // Other car must be ahead in the same lane
//         if(v.isInLane && v.lane == ego.lane && v.s > ego.s)
//         {
//             double dist = distance(ego.x, ego.y, v.x, v.y);
//             if(dist < min_distance)
//             {
//                 min_distance = dist;
//             }
//         }
//     }

//     double diff = VEHICLE_DISTANCE_THRESHOLD_METERS - min_distance;
//     return weight * (1 - exp(- abs(diff)));
// }




/**
 * @brief Measures the distance to the goal at the end of our trajectory
 * 
 * @param ego 
 * @param others 
 * @param trajectory 
 * @param state 
 * @param weight 
 * @return double the distance to the goal at the end of our trajectory
 */
double futureDistanceToGoalCostFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                                        const State& state, const double& weight)                                             
{
    int traj_size = trajectory.size();
    double diff = GOAL_POSITION_METERS - trajectory.ss[traj_size - 1];

    // cout << "** DISTANCE TO GOAL = " << diff << endl;
    return weight * (1 - exp(- abs(diff / GOAL_POSITION_METERS)));
}
