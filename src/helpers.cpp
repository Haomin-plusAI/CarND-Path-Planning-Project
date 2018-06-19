#include <iostream>
#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "helpers.h"

using namespace std;

// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

/**
 * 
 * @returns the angle theta in radians, based on the supplied velocity components
 */
double getTheta(double vx, double vy)
{
	return atan2(vy, vx);
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

/**
 * 
 * @returns the calculated lane index
 */
int calculateLane(double d, double lane_spacing, double lane_inside_offset)
{
	double calibrated_d = d - lane_inside_offset;
	if (calibrated_d < 0.0)
	{
		return -1;
	}
	return (int)floor(calibrated_d / lane_spacing);
}

bool isWithinLane(double d, double lane_spacing, double lane_inside_offset)
{
	double calibrated_d = d - lane_inside_offset;
	if (calibrated_d < 0.0)
	{
		return false;
	}

	int target_lane = (int)floor(d / lane_spacing);
	int calibrated_lane = (int)floor(calibrated_d / lane_spacing);
	return target_lane == calibrated_lane;
}

bool isLaneValid(int lane)
{
	return lane >= 0 && lane < LANES_COUNT;
}

double getLaneCenterFrenet(int lane)
{
	return 2.0 + 4.0 * lane;
}

double milesPerHourToKmPerHour(double mph)
{
	return mph * 1.609344;
}

double KmPerHourToMetersPerSecond(double kmh)
{
	return (kmh * 1000.0) / 3600.0;
}
