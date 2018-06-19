#ifndef COST_FUNCTIONS_H
#define COST_FUNCTIONS_H
#include <iostream>
#include <vector>
#include <functional>
#include <math.h>
#include "vehicle.h"
#include "trajectory.h"
#include "state_machine.h"
#include "collision_detector.h"
#include "helpers.h"

using namespace std;

typedef function<double (const Vehicle&, const vector<Vehicle>&,  const Trajectory&, const State&, const double&)> CostFunction;


double speedCostFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                         const State& state, const double& weight);


double centerOfLaneDistCostFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                                    const State& state, const double& weight);


double laneChangeCostFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                              const State& state, const double& weight);
                              

double distanceToClosestCarAheadCostFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                                             const State& state, const double& weight);


double longitudinalDistanceToClosestAdjacentCarFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                                                        const State& state, const double& weight);


double distanceToClosestCarAheadFutureLaneCostFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                                                       const State& state, const double& weight);                                                                                                     


double averageLaneSpeedDiffCostFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                                        const State& state, const double& weight);



double speedDifferenceWithClosestCarAheadCostFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                                                      const State& state, const double& weight);


double lanesAverageForwardSpeedCarsAhead(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                                                      const State& state, const double& weight);




double collisionTimeCostFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                                 const State& state, const double& weight);




double futureDistanceToGoalCostFunction(const Vehicle& ego, const vector<Vehicle>& others,  const Trajectory& trajectory, 
                                        const State& state, const double& weight);                                         


#endif