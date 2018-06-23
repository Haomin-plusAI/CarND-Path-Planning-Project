# Write up: Path Planning Project

* Overwhelming
* Didn't really know where to start

# Experiments

## Attempting to drive the car within its lane

* Trying to maintain the same value for lateral position _d_ in Frenet coordinates
* Simply adding an offset to longitudinal position _s_ in Frenet coordinates
* Car could stay in its lane but breached speed limit and also was colliding with other vehicles in same lane

```
double s_inc = 0.2;
for(int i = 0; i < 50; ++i)
{
    auto xy = getXY(car_s + s_inc * (i+1), car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);
}
```

## Attempting to drive vehicle in lane without colliding with other vehicles

This is achieved by making sure we have the appropriate cost functions with the suitable weights,
so that it is more costly to drive faster than the vehicle in front as this would reduce the distance
between our two vehicles and cause an earlier potential collision.

However, if our ego vehicle matches the speed of the vehicle ahead and the distance between the two
is higher than our configured threshold, then the ego vehicle will stick to the current lane and drive 
at a speed that matches that of the vehicle ahead



## Determining whether reference vehicle will collide with another car

The `CollisionDetector` class performs the collision prediction part of the pipeline, using the current 
trajectory computed for our ego vehicle along with the current position and angular velocity of the other 
vehicle. We maintain the following assumptions:
* angular velocities vx and vy are constant for the other vehicle
* acceleration of the other vehicle is 0

A collision is identified when the distance between our ego vehicle and the other car is less than 1 meter. 
The `predictCollision` class will then return the x, y and the timestep of the forecast collision.


# Create smoother trajectory

The map waypoints we are given are quite sparse and can lead to very 
"angular" trajectories generated when we try to convert from Frenet back to 
real world coordinates. This in turn causes sudden spikes in acceleration 
and jerk.
As the function toRealWorld(s, d) -> (x, y) uses basic interpolation between
two waypoints to find the best approximate values for x and y, we always run
the risk of generating a non-smooth trajectory.

What can we do to improve upon this. From some of the projects in previous 
terms, we have seen that lines derived from a polynomial tend to produce 
very smooth trajectories. Therefore we should employ this technique instead
of the basic interpolation that is currently being used. We resort to using
 splines created by using the position s in Frenet coordinates to obtain 
 the real-world coordinates _x_, _y_, and offsets _dx_ and _dy_. We then 
 plug in this formula to obtain the closest real-world coordinates
 
 ```
 x = spline_s_x(s) + d * spline_s_dx(s)
 y = spline_s_y(s) + d * spline_s_dy(s)
 ```

 Now we can see an _remarkably_ smooth our trajectory has become. 

 (INSERT IMAGE OF SMOOTH TRAJECTORY)


 # Speed limit

 I initially had set the maximum speed to be at 22 meters per second, which roughly equates to 50MPH but 
 some sections of my jerk minimised path sometimes exceeded this speed, so I decided to bring down the maximum
 achievable speed to around 21 meters per second, which is close to 47MPH.
 I've noticed that leaving this safety buffer helps in never violating the speed limit set in the simulator.
 The downside is also somewhat mitigated as we are generally less than 2MPH from the maximum authorised speed.


 # How to deal with wrap around in track

 The track is close to 7KM long and we have waypoints sparsely distributed, starting with a ~30m spacing but getting larger over time.
 Even with splines, this causes a problem when the track wraps around and the  longitudinal _s_ frenet coordinate moves from 69XX to 1.
 We actually have our last waypoint s=~6914.14, while the max s value is 6945.554. We therefore have a gap at the end of our waypoints
 plug it by assuming:
 - x(s_last) = x(s_first)
 - y(s_last) = y(s_first)
 - dx(s_last) = dx(s_first)
 - dy(s_last) = dy(s_first)
 Naturally,m s_last=6945.554, while s_first=~0.0

 This "trick" addresses the inconsistency observed in the trajectory when our car comes back full circle. 

 Moreover, as we compute multiple trajectories more than one second ahead of time, our predicted s values may
 be greater than the max s value of the track, which is not feasible. We therefore resort to applying the 
 _fmod_ operation to all predicted s_values to make sure they are capped between 0 and 6945.554.

 ```
 vector<double> Map::toRealWorldXY(double s, double d)
{
	s = fmod(s, MAX_TRACK_S);
	// Use the spline we have created to get a smoother path
	double x = sp_x_s(s) + d * sp_dx_s(s);
	double y = sp_y_s(s) + d * sp_dy_s(s);

	return {x, y};
}
 ```


# Cost functions

Divided between comfort and safety cost functions. Safety functions are much more life critical than comfort ones I reckon.

## Change lane cost functions
- closest car behind on next lane must be a good 15 meters behind 
- closest car ahead  on next lane must be more than 30 meters ahead ideally 
- if car behind in next lane is more than 15 meters behind then it's fine
- distance to closest car ahead cost function to revise (when there is no car ahead in the next lane then we should change lanes)

## Avoiding collision
- right now ego car collides too much
- should avoid collision RIGHT AHEAD by slowing down





