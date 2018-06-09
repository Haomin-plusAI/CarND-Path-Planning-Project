#include <iostream>
#include <vector>
#include <math.h>
#include "behaviour.h"
#include "vehicle.h"
#include "trajectory.h"
#include "state_machine.h"
#include "collision_detector.h"


using namespace std;

Behaviour::Behaviour(){
    // Default state
    this->state.s_state = LongitudinalState::ACCELERATE;
    this->state.d_state = LateralState::STAY_IN_LANE;
}

vector<State> Behaviour::update(const Vehicle& ego, const vector<Vehicle> others, Trajectory& current_trajectory)
{
    
    // Identify collisions now and in 1 second
    CollisionDetector cdetector = CollisionDetector(current_trajectory);
    
    // TODO put that 0.02 into a constant
    vector<State> potential_next_states;
    
    for(int i = 0; i < others.size(); ++i)
    {
        Vehicle v = others[i];
        Collision collision = cdetector.predictCollision(v, 0.02);    

        if(collision.willCollide)
        {
            // This path is not possible because of future collision
            

            // Return a set of states the car could take
            State s1 = State();            
            s1.s_state = LongitudinalState::DECELERATE;
            s1.d_state = LateralState::STAY_IN_LANE;
            potential_next_states.push_back(s1);

            State s2 = State();            
            s2.s_state = LongitudinalState::MAINTAIN_COURSE;
            s2.d_state = LateralState::STAY_IN_LANE;
            potential_next_states.push_back(s2);          

            return potential_next_states;      
        }
    }

    State s1 = State();            
    s1.s_state = LongitudinalState::MAINTAIN_COURSE;
    s1.d_state = LateralState::STAY_IN_LANE;
    potential_next_states.push_back(s1);                

    State s2 = State();            
    s2.s_state = LongitudinalState::ACCELERATE;
    s2.d_state = LateralState::STAY_IN_LANE;
    potential_next_states.push_back(s2);                


    return potential_next_states;
}