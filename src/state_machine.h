#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

enum LongitudinalState
{
    ACCELERATE,
    DECELERATE,
    MAINTAIN_COURSE,
    STOP
};

enum LateralState
{
    STAY_IN_LANE,
    PREPARE_CHANGE_LANE_LEFT,
    PREPARE_CHANGE_LANE_RIGHT,
    CHANGE_LANE_LEFT,
    CHANGE_LANE_RIGHT
};

class State
{
    LongitudinalState s_state;
    LateralState d_state;

    
};

#endif