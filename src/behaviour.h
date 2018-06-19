#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H
#include <iostream>
#include <vector>
#include "vehicle.h"
#include "trajectory.h"
#include "state_machine.h"

using namespace std;

class Behaviour{

    public:

        Behaviour();
        
        

        Trajectory nextTrajectory(const Vehicle &ego, const vector<Vehicle>& others,
                                  vector<double>& previous_path_x, vector<double>& previous_path_y);

        vector<State> update(const Vehicle& ego, const vector<Vehicle> others, Trajectory& current_trajectory);
        
        void updateState(State state );

        ~Behaviour();

    private:
        StateMachine state_machine;
        Trajectory trajectory;
        int current_timestep;        
        int lock_timestep;        

};


#endif