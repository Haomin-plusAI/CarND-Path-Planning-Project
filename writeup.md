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


