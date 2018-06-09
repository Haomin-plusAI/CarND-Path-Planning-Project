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
        
        vector<State> update(const Vehicle& ego, const vector<Vehicle> others, Trajectory& current_trajectory);

        ~Behaviour();

    private:
        State state;

};


#endif