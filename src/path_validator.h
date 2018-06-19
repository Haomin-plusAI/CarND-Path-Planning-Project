#ifndef PATH_VALIDATOR_H
#define PATH_VALIDATOR_H
#include "vehicle.h"
#include "state_machine.h"
#include "trajectory.h"
#include <vector>

using namespace std;

enum PathValidationStatus
{
    VALID,
    OUTSIDE_OF_LANE,
    VELOCITY_TOO_LOW_FOR_LANE_CHANGE,
    COLLISION_VEHICLE_AHEAD,
    COLLISION_VEHICLE_ADJACENT,
    AVERAGE_SPEED_BELOW_THRESHOLD,
    VELOCITY_ABOVE_THRESHOLD,
    TOTAL_ACCELERATION_ABOVE_THRESHOLD
};

class PathValidator
{
  public:
    PathValidator();
    ~PathValidator();

    PathValidationStatus validate(const Vehicle &v, vector<Vehicle> others, const State &state, const Trajectory &trajectory, int from_point) const;
};

#endif
