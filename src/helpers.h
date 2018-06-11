#ifndef HELPERS_H
#define HELPERS_H

#include <iostream>
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

using namespace std;

const double MAX_SPEED_METERS_PER_SECOND = 22.0;
const double CONTROLLER_UPDATE_RATE_SECONDS = 0.02;

const double VEHICLE_DISTANCE_THRESHOLD_METERS = 25;

const double VEHICLE_MIN_SECURITY_DISTANCE_METERS = 10.0;

const double VEHICLE_COLLISION_THRESHOLD_METERS = 5.0;

// TODO It is better to make this configurable
const double GOAL_POSITION_METERS = 6945.554;

const int LANES_COUNT = 3;
const double DEFAULT_LANE_SPACING = 4.0;
const double DEFAULT_LANE_INSIDE_OFFSET = 1.5;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x);
double rad2deg(double x);

double getTheta(double vx, double vy);

/**
 * 
 * @returns the calculated lane index
 */ 
int calculateLane(double d, double lane_spacing, double lane_inside_offset);

bool isWithinLane(double d, double lane_spacing, double lane_inside_offset);

/**
 * @brief Checks whether the lane id is a valid one
 * 
 * @param lane the lane id
 * @return true if the id is within our range of admissible lanes
 * @return false if the id is outside of our admissible lane range
 */
bool isLaneValid(int lane);

/**
 * @brief Get the appropriate lane center value (d_center) in Frenet coordinates
 * 
 * @param lane the lane id
 * @return double the lane center in frenet coordinates
 */
double getLaneCenterFrenet(int lane);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

double milesPerHourToKmPerHour(double mph);

double KmPerHourToMetersPerSecond(double kmh);

#endif