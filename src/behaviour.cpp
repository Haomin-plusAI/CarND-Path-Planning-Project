#include <iostream>
#include <vector>
#include <math.h>
#include "behaviour.h"
#include "vehicle.h"
#include "trajectory.h"
#include "state_machine.h"
#include "path_generator.h"
#include "path_validator.h"
#include "cost_functions.h"
#include "helpers.h"
#include "collision_detector.h"



using namespace std;


Behaviour::Behaviour(){    
    this->current_timestep = 0;    
    this->lock_timestep = 0;  
    trajectory = Trajectory();  
}

Trajectory Behaviour::nextTrajectory(const Vehicle &ego, const vector<Vehicle>& vehicles, 
                                    vector<double>& previous_path_x, vector<double>& previous_path_y)
{       
    Trajectory chosen_trajectory;
    State chosen_state;
    double lowest_cost = 10000;
    bool state_chosen  = false;

    PathValidator path_validator = PathValidator();
    
    if(this->current_timestep == 0)
    {
        this->trajectory.add(ego.x, ego.y, ego.s, 0.0, 0.0, 0.0, ego.d, 0.0, 0.0, 0.0, ego.theta);              
        vector<State> next_states = this->update(ego, vehicles, this->trajectory);
        
        // TODO Move PathGenerator into behvaviour layer
        PathGenerator path_gen = PathGenerator(ego, this->trajectory);
        for(const State& state : next_states)
        {
            auto paths = path_gen.generatePaths(state, ego, this->trajectory, 0, 1, 1.0);                  
                  
            for(auto& path: paths)
            {
                PathValidationStatus path_status = path_validator.validate(ego, vehicles, state, path, 1);
                cout << "*** PATH VALIDATION STATUS =  for state (" << state.s_state << ", " << state.d_state << ") =" 
                        << path_status << endl;
                if(path_status != PathValidationStatus::VALID)
                {
                    continue;
                }

                CostFunction lane_center_cost_fn = centerOfLaneDistCostFunction;
                double lane_center_cost = centerOfLaneDistCostFunction(ego, vehicles, path, state, 5.0);
                
                CostFunction cost_speed_fn = speedCostFunction;
                double cost_speed = cost_speed_fn(ego, vehicles, path, state, 1.0);
                
                CostFunction avg_speed_lane_diff_fn = averageLaneSpeedDiffCostFunction;
                double avg_speed_lane_diff_cost = avg_speed_lane_diff_fn(ego, vehicles, path, state, 10.0);

                CostFunction dist_cars_cost_fn = distanceToClosestCarAheadCostFunction;
                double cost_dist_cars = dist_cars_cost_fn(ego, vehicles, path, state, 5.0);
                
                CostFunction change_lane_cost_fn = laneChangeCostFunction;
                double change_lane_cost = change_lane_cost_fn(ego, vehicles, path, state, 10.0);
                
                CostFunction future_dist_to_goal_cost_fn = futureDistanceToGoalCostFunction;
                double future_dist_to_goal_cost = future_dist_to_goal_cost_fn(ego, vehicles, path, state, 1.0);
                
                CostFunction speed_diff_to_car_ahead_fn = speedDifferenceWithClosestCarAheadCostFunction;
                double speed_diff_to_car_ahead_cost = speed_diff_to_car_ahead_fn(ego, vehicles, path, state, 100.0);
                
                CostFunction collision_time_cost_fn = collisionTimeCostFunction;
                double collision_time_cost = collision_time_cost_fn(ego, vehicles, path, state, 100.0);
                
                CostFunction dist_car_future_lane_cost_fn = distanceToClosestCarAheadFutureLaneCostFunction;
                // double dist_car_future_lane_cost = dist_car_future_lane_cost_fn(ego, vehicles, path, state, 100.0);
                double dist_car_future_lane_cost = 0.0;
                
                
                CostFunction lon_dist_adjacent_car_cost_fn = longitudinalDistanceToClosestAdjacentCarFunction;
                double lon_dist_adjacent_car_cost = lon_dist_adjacent_car_cost_fn(ego, vehicles, path, state, 1000.0);

                double final_cost = lane_center_cost
                                    + cost_speed 
                                    + avg_speed_lane_diff_cost
                                    + cost_dist_cars 
                                    + change_lane_cost 
                                    + future_dist_to_goal_cost + speed_diff_to_car_ahead_cost
                                    + collision_time_cost + dist_car_future_lane_cost
                                    + lon_dist_adjacent_car_cost;

                cout << left << "(" << state.s_state << "," << state.d_state << ")" 
                        << ":" << state.current_lane << "->" << state.future_lane << endl;                                  
                cout << left << setw(14) << setfill(' ') << "   Ego Lane: " << ego.lane;
                cout << left << "| " << setw(13) << setfill(' ') << lane_center_cost;
                cout << left << "| " << setw(13) << setfill(' ') << cost_speed;
                cout << left << "| " << setw(13) << setfill(' ') << avg_speed_lane_diff_cost;
                cout << left << "| " << setw(13) << setfill(' ') << cost_dist_cars;
                cout << left << "| " << setw(13) << setfill(' ') << change_lane_cost;
                cout << left << "| " << setw(13) << setfill(' ') << dist_car_future_lane_cost;
                cout << left << "| " << setw(13) << setfill(' ') << lon_dist_adjacent_car_cost;
                cout << left << "| " << setw(13) << setfill(' ') << future_dist_to_goal_cost;
                cout << left << "| " << setw(13) << setfill(' ') << speed_diff_to_car_ahead_cost;
                cout << left << "| " << setw(13) << setfill(' ') << collision_time_cost;
                cout << left << "| " << setw(13) << setfill(' ') << final_cost;
                cout << endl;

                if(final_cost < lowest_cost)
                {
                    lowest_cost = final_cost;
                    chosen_trajectory = path;
                    chosen_state = state;
                }                    
            }
        }

        cout << "* FIRST TIME - Chosen state: (" << chosen_state.s_state
             << "," << chosen_state.d_state << ")" << endl; 
        // Make sure to remove the first position, since the car is already "there"
        chosen_trajectory.removeFirstN(1);

        this->trajectory = chosen_trajectory;
        this->updateState(chosen_state);        
    }


    return chosen_trajectory;
}

vector<State> Behaviour::update(const Vehicle& ego, const vector<Vehicle> others,
                                Trajectory& current_trajectory)
{
    if(this->current_timestep == 0)
    {        
        // This is the initialisation step
        int start_lane = calculateLane(current_trajectory.ds[0], DEFAULT_LANE_SPACING, DEFAULT_LANE_INSIDE_OFFSET);
        int finish_lane = calculateLane(current_trajectory.ds[current_trajectory.size() - 1], DEFAULT_LANE_SPACING, DEFAULT_LANE_INSIDE_OFFSET);

        State initial_state = State(LongitudinalState::ACCELERATE, LateralState::STAY_IN_LANE, start_lane, finish_lane);
        this->state_machine = StateMachine(initial_state);
    }
    ++this->current_timestep;     

    auto next_states = this->state_machine.nextPossibleStates();    

    vector<State> reachable_next_states;    
    for(const State& next_state : next_states)
    {
        
        
        if(!isLaneValid(next_state.current_lane) || !isLaneValid(next_state.future_lane))
        {
            continue;
        }
        reachable_next_states.push_back(next_state);
    }

    return reachable_next_states;


    // this->state.current_lane = ego.lane;
    

    // Identify collisions now and in 1 second
    // CollisionDetector cdetector = CollisionDetector(current_trajectory);
    
    // TODO put that 0.02 into a constant
    // vector<State> potential_next_states;
    // cout << "*** LANE = " << ego.lane << endl;
    // for(int i = 0; i < others.size(); ++i)
    // {
    //     Vehicle v = others[i];
    //     Collision collision = cdetector.predictCollision(v, 0.02);    

    //     if(collision.willCollide)
    //     {
    //         // This path is not possible because of future collision
    //         cout << ">> Ego will collide with vehicle " << collision.v.id 
    //              << " (ego speed: " << current_trajectory.averageSpeed(1.0)
    //              << " vehicle speed: " << v.getSpeed()  << " )"
    //              << " at timestep " << collision.collision_timestep
    //              << " in " << collision.collision_timestep * 0.02 << " seconds" << endl;

    //         // Return a set of states the car could take
    //         State s1 = State();  
    //         s1.current_lane = ego.lane;          
    //         s1.future_lane = ego.lane;          
    //         s1.s_state = LongitudinalState::DECELERATE;
    //         s1.d_state = LateralState::STAY_IN_LANE;
    //         potential_next_states.push_back(s1);

    //         State s2 = State();    
    //         s2.current_lane = ego.lane;          
    //         s2.future_lane = ego.lane;                  
    //         s2.s_state = LongitudinalState::MAINTAIN_COURSE;
    //         s2.d_state = LateralState::STAY_IN_LANE;
    //         potential_next_states.push_back(s2);         

    //         // TODO Put logic elsewhere
    //         // if(ego.lane - 1 >= 0)
    //         // {
    //         //     State s3 = State();  
    //         //     s3.current_lane = ego.lane;          
    //         //     s3.future_lane = ego.lane - 1;                    
    //         //     s3.s_state = LongitudinalState::MAINTAIN_COURSE;
    //         //     s3.d_state = LateralState::CHANGE_LANE_LEFT;
    //         //     potential_next_states.push_back(s3);         
    //         // }

    //         // if(ego.lane + 1 < LANES_COUNT)
    //         // {
    //         //     State s3 = State();  
    //         //     s3.current_lane = ego.lane;          
    //         //     s3.future_lane = ego.lane + 1;                 
    //         //     s3.s_state = LongitudinalState::MAINTAIN_COURSE;
    //         //     s3.d_state = LateralState::CHANGE_LANE_RIGHT;
    //         //     potential_next_states.push_back(s3);         
    //         // }
            
    //         // cout << "*** (1) NEXT STATES COUNT = " << potential_next_states.size() << endl;
    //         return potential_next_states;      
    //     }
    // }
    
    // State s1 = State();            
    // s1.s_state = LongitudinalState::MAINTAIN_COURSE;
    // s1.d_state = LateralState::STAY_IN_LANE;
    // potential_next_states.push_back(s1);                

    // State s2 = State();            
    // s2.s_state = LongitudinalState::ACCELERATE;
    // s2.d_state = LateralState::STAY_IN_LANE;
    // potential_next_states.push_back(s2);      

    // TODO Put logic elsewhere
    // if(ego.lane - 1 >= 0)
    // {
    //     State s3 = State();  
    //     s3.current_lane = ego.lane;          
    //     s3.future_lane = ego.lane - 1;                    
    //     s2.s_state = LongitudinalState::MAINTAIN_COURSE;
    //     s3.d_state = LateralState::CHANGE_LANE_LEFT;
    //     potential_next_states.push_back(s3);         
    // }

    // if(ego.lane + 1 < LANES_COUNT)
    // {
    //     State s4 = State();  
    //     s4.current_lane = ego.lane;          
    //     s4.future_lane = ego.lane + 1;                 
    //     s4.s_state = LongitudinalState::MAINTAIN_COURSE;
    //     s4.d_state = LateralState::CHANGE_LANE_RIGHT;
    //     potential_next_states.push_back(s4);         
    // }
    
    // cout << "*** (2) NEXT STATES COUNT = " << potential_next_states.size() << endl;          


    // return potential_next_states;
}

void Behaviour::updateState(State new_state)
{
    
    const State &current_state = this->state_machine.getCurrentState();    
    cout << "********** current state D = " << current_state.d_state << endl;
    cout << "********** new state D = " << new_state.d_state << endl;
    if( (current_state.d_state == LateralState::PREPARE_CHANGE_LANE_LEFT &&
        new_state.d_state == LateralState::CHANGE_LANE_LEFT)
        ||
        (current_state.d_state == LateralState::PREPARE_CHANGE_LANE_RIGHT &&
        new_state.d_state == LateralState::CHANGE_LANE_RIGHT)
        )                        
    {
        this->lock_timestep = this->current_timestep + 50;
        // tlock = 50;
        //   cout << "*********************************************************************************" << endl;
        // cout << "*********************************************************************************" << endl;
        // cout << "*********************************************************************************" << endl;
        // cout << "*********************************************************************************" << endl;
        // cout << "*********************************************************************************" << endl;
        cout << "*** FREEZING state updates for " << this->lock_timestep << "timesteps";
    }
    
    this->state_machine.updateState(new_state, this->lock_timestep);
}

Behaviour::~Behaviour(){}