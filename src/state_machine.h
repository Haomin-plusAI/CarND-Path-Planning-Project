#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <iostream>
#include <vector>
#include "collision_detector.h"

using namespace std;

enum LongitudinalState
{
    ACCELERATE = 0,
    DECELERATE = 1,
    MAINTAIN_COURSE = 2,
    STOP = 3
};


enum LateralState
{
    STAY_IN_LANE = 0, 
    PREPARE_CHANGE_LANE_LEFT = 1,
    PREPARE_CHANGE_LANE_RIGHT = 2,
    CHANGE_LANE_LEFT = 3,
    CHANGE_LANE_RIGHT = 4
};

class State
{
    public:
        State();
        ~State();

        LongitudinalState s_state;
        LateralState d_state;
                
        int current_lane;
        int future_lane;
        
};

#endif