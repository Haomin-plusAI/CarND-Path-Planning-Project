#include <iostream>
#include <vector>
#include <math.h>
#include "behaviour.h"
#include "vehicle.h"
#include "trajectory.h"
#include "state_machine.h"
#include "helpers.h"
#include "collision_detector.h"



using namespace std;


Behaviour::Behaviour(){    
    this->current_timestep = 0;    
    this->lock_timestep = 0;    
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