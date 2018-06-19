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
    State(LongitudinalState lon_state, LateralState lat_state, int current_lane, int future_lane);
    ~State();

    LongitudinalState s_state;
    LateralState d_state;

    int current_lane;
    int future_lane;
};

class StateMachine
{
  public:
    StateMachine();
    StateMachine(State state);
    ~StateMachine();

    /**
         * @brief Computes the next possible states from the current underlying state
         * Not all states may be feasible and therefore feasibility checks must be 
         * performed upstream.
         * 
         * @return vector<State> the vector of possible next states
         */
    vector<State> nextPossibleStates();

    /**
         * @brief Updates the internally stored state with the next state along with
         * a "lock" timer. 
         * 
         * @param next_state the next state desired
         * @param keep_until_timestep how long to keep this state for. If the state is to be
         * objectly re-evaluated at the next update pass -1.
         */
    void updateState(State next_state, int keep_until_timestep);

    const State getCurrentState() const;

  private:
    State current_state;
    int timestep_lock;
    int current_timestep;
};

#endif
