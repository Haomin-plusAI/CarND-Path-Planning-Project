# Write up - Path Planning Project


![Path Planning On Highway](media/path_planning_cover.jpeg)


Planning an optimal path is both _safe_ and _efficient_ is one of the hardest problems in the development of
an autonomous vehicle. This step, known as _Path Planning_, is still tackled in the industry an in research.
The reason why Path Planning is such a complex task is because it involves all components of a self-driving
vehicle, ranging from the low level actuators, the sensors which are fused to create a "snapshot" of the world,
along with the localisation and prediction modules to understand precisely where we are and what actions different
entities in your world (other vehicles, humans, animals, etc) are more likely to take in the next few seconds. One
other obivous component is a trajectory generator that can compute a candidate trajectory to be evaluated by the
planner.

In this post, we will be focusing on describing how we implemented a highway path planner that is able to generate
safe and efficient trajectories in a simulator, using jerk minimised trajectories.



## Tackling The Project

At first, I was overwhelmed by this project: there were so many new concepts we had covered in 
the lessons and so
many elements to keep in mind when designing such a path planner. I however, quickly decided on 
which trajectory generation technique I wanted to focus on. The two recommended options were:

1. trajectory generation in (x, y) coordinates
2. trajectory generation in (s, d) frenet coordinates by using a quintic polynomial to compute a  jerk minimal path

The first approach was the most common among students but I decided to go for the second method 
which felt more challenging but also more rewarding, in the sense that I hoped I would learn more
about trajectory generation.

### Project Structure

Moreover, I quickly decided to divide responsibility for specific tasks to different classes
and store information about critical entities such as vehicles and trajectories in another class. I thus created the following core classes:

* _Vehicle_ : a class to store information about a given vehicle identified by the sensor fusion module or our own vehicle
* _Map_ a [singleton](https://en.wikipedia.org/wiki/Singleton_pattern) class that encapsulates information about our current location along with the waypoints to convert from real-world to Frenet coordinates and vice-versa
* _StateMachine_: a simple class that stores all possible states as well as the allowed transitions from a current state to a next state
* _PathGenerator_: a very important class whose sole role is to generate trajectories using the quintic polynomial approach to Frenet coordinates
* _Trajectory_: a class where we store a given trajectory in real-word and Frenet coordinates, and extract information about it such as average speed, etc
* _PathValidator_: a useful class to ensure a given path does not violate constraints such the legal speed limit and maximum acceleration and total jerk
* _CollisionDetector_: a useful class to foresee future collisions
* _Behaviour_: **the coordinating class that takes in information from all components of our self-driving car and ouputs the safest and most efficient state and trajectory for the next 1.7 seconds**

Moreover, the _helpers_ and _cost_functions_ files contain respectively a myriad of helper 
functions to make our task easier and a multitude of cost functions to score a path
based on many parameters.

### Longitudinal And Lateral States

I took inspiration on the Frenet coordinates to devise this method. I decided to to split a given
state into its _longitudinal_ and _lateral_ component. The reason for doing so is that I believe
it simplifies how we think about driving on a highway with potential lane changes.

**Basically the lateral state dictates the potential next states we could find ourselves in, while
the cost functions may select a longitudinal state over another.** 

We have the following states:

```
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

```

Our final state machine is also quite self-explanatory:
```
	vector<State> future_states;
    switch (this->current_state.d_state)
    {
    case LateralState::STAY_IN_LANE:
        future_states.push_back(State(LongitudinalState::MAINTAIN_COURSE,
                                      LateralState::STAY_IN_LANE,
                                      this->current_state.current_lane,
                                      this->current_state.current_lane));
        future_states.push_back(State(LongitudinalState::ACCELERATE,
                                      LateralState::STAY_IN_LANE,
                                      this->current_state.current_lane,
                                      this->current_state.current_lane));
        future_states.push_back(State(LongitudinalState::DECELERATE,
                                      LateralState::STAY_IN_LANE,
                                      this->current_state.current_lane,
                                      this->current_state.current_lane));
        // TODO do we ever want to return LongitudinalState::STOP ??

        // When evaluating whether to change lane, always stay at the same speed
        future_states.push_back(State(LongitudinalState::MAINTAIN_COURSE,
                                      LateralState::PREPARE_CHANGE_LANE_LEFT,
                                      this->current_state.current_lane,
                                      this->current_state.current_lane - 1));
        future_states.push_back(State(LongitudinalState::ACCELERATE,
                                      LateralState::PREPARE_CHANGE_LANE_LEFT,
                                      this->current_state.current_lane,
                                      this->current_state.current_lane - 1));
        future_states.push_back(State(LongitudinalState::DECELERATE,
                                      LateralState::PREPARE_CHANGE_LANE_LEFT,
                                      this->current_state.current_lane,
                                      this->current_state.current_lane - 1));

        future_states.push_back(State(LongitudinalState::MAINTAIN_COURSE,
                                      LateralState::PREPARE_CHANGE_LANE_RIGHT,
                                      this->current_state.current_lane,
                                      this->current_state.current_lane + 1));
        future_states.push_back(State(LongitudinalState::ACCELERATE,
                                      LateralState::PREPARE_CHANGE_LANE_RIGHT,
                                      this->current_state.current_lane,
                                      this->current_state.current_lane + 1));
        future_states.push_back(State(LongitudinalState::DECELERATE,
                                      LateralState::PREPARE_CHANGE_LANE_RIGHT,
                                      this->current_state.current_lane,
                                      this->current_state.current_lane + 1));

        break;
    case LateralState::PREPARE_CHANGE_LANE_LEFT:
        future_states.push_back(State(LongitudinalState::MAINTAIN_COURSE,
                                      LateralState::CHANGE_LANE_LEFT,
                                      this->current_state.future_lane,
                                      this->current_state.future_lane));
        future_states.push_back(State(LongitudinalState::MAINTAIN_COURSE,
                                      LateralState::STAY_IN_LANE,
                                      this->current_state.current_lane,
                                      this->current_state.current_lane));
        future_states.push_back(State(LongitudinalState::DECELERATE,
                                      LateralState::STAY_IN_LANE,
                                      this->current_state.current_lane,
                                      this->current_state.current_lane));
        break;

    case LateralState::PREPARE_CHANGE_LANE_RIGHT:
        future_states.push_back(State(LongitudinalState::MAINTAIN_COURSE,
                                      LateralState::CHANGE_LANE_RIGHT,
                                      this->current_state.future_lane,
                                      this->current_state.future_lane));
        future_states.push_back(State(LongitudinalState::MAINTAIN_COURSE,
                                      LateralState::STAY_IN_LANE,
                                      this->current_state.current_lane,
                                      this->current_state.current_lane));
        future_states.push_back(State(LongitudinalState::DECELERATE,
                                      LateralState::STAY_IN_LANE,
                                      this->current_state.current_lane,
                                      this->current_state.current_lane));
        break;

    default:       
        future_states.push_back(State(LongitudinalState::MAINTAIN_COURSE,
                                      LateralState::STAY_IN_LANE,
                                      this->current_state.current_lane,
                                      this->current_state.current_lane));       
    }
    return future_states;
```


### Cost Functions

All our cost functions adhere to the following interface we defined in _cost_functions.h_

```
typedef function<double (const Vehicle&, const vector<Vehicle>&,  const Trajectory&, const State&, const double&)> CostFunction;
```

This makes adding a cost function very trivial. We defined the following cost functions, where
the weight is entirely parameterizable:

* _speedCostFunction_: a function that penalises our vehicle if it goes slowly
* _centerOfLaneDistCostFunction_: a function that penalises our vehicle if its trajectory ends outside of the center of the targeted laned
* _laneChangeCostFunction_: a function that _always_ penalises a lane change since those are usuallyu more dangerous than driving in the same lane
* _distanceToClosestCarAheadCostFunction_: a function that penalises our car the closest it is to the car ahead
* _distanceToClosestCarAheadFutureLaneCostFunction_: a function that penalises a vehicle if it is 
too close to vehicles ahead or behind when operating a lane change
* _averageLaneSpeedDiffCostFunction_: a function that penalises the lane the vehicle wants to be in based on the average speed of the vehicles ahead in this lane
* _speedDifferenceWithClosestCarAheadCostFunction_: a cost function that penalises our vehicle if it is going slowler than the vehicle ahead
* _collisionTimeCostFunction_: a function that penalises our trajectory if it leads to a potential collusion with another vehicle
* _futureDistanceToGoalCostFunction_: a function that penalises our trajectory the further its end point is to the goal (i.e. the end of the track at s=6945.554)

We tried many different weight configurations but ultimately decided that since we did not wish to
compromise on safety we attribute the _collisionTimeCostFunction_ the highest weight with 10000. 
Other cost function weights vary widely but we gave for instance very little weight to 
_speedCostFunction_ since high speed is a nice to have but is nowhere near as critical as **no-collision**.


## Tricky Problems

### Creating Smoother Trajectories

The map waypoints we are given are quite sparse and can lead to very 
"angular" trajectories generated when we try to convert from Frenet back to 
real world coordinates. This in turn causes sudden spikes in acceleration 
and jerk.
As the function toRealWorld(s, d) -> (x, y) uses basic interpolation between
two waypoints to find the best approximate values for x and y, we always run
the risk of generating a non-smooth trajectory.

![Waypoints Interpolation Produce Terrible Path](media/path_planning_vehicle_not_smooth_switch_lane.gif)

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

 ![Smooth Trajectory With Splines](media/path_planning_smooth_trajectory.gif)


 ### Speed limit

 I initially had set the maximum speed to be at 22 meters per second, which roughly equates to 50MPH but 
 some sections of my jerk minimised path sometimes exceeded this speed, so I decided to bring down the maximum
 achievable speed to around 21 meters per second, which is close to 47MPH.
 I've noticed that leaving this safety buffer helps in never violating the speed limit set in the simulator.
 The downside is also somewhat mitigated as we are generally less than 2MPH from the maximum authorised speed.


 ### How to deal with wrap around in track

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

## Final Results

The current path planner performs quite well and is 
enables the vehicle to drive around the track 
_multiple_ times. It can however be improved by 
tweaking weights more and improving some of the cost 
functions. Moreover, we believe that by incorporating
some machine learning into our prediction layer we
can eliminate some edge cases that could lead to 
collisions. One interesting behaviour of our planner
is that it is able to quickly change lanes multiple 
times . We initially thought this was a bug from our 
final state machine design, but it turns out to be an 
interesting side effect!

![Vehicle Change Lanes Twice](media/path_planning_smooth_lane_change.gif)



## Improvements

The current path planner is relatively conservative and is not optimised for speed. This means that while the velocity 
can at times go as high as 48 MPH (~77.2 KPH), it generally stays below this speed, and does almost never attain the
legal speed limit of 50 MPH (~80.4 KPH). That's a trade-off we are happy to accept for one but work should be done to
enable the car to drive at speeds closer to 50 MPH.

Moreover, the planner only takes into account the vehicle's adjacent lane and therefore never "sees" when a 
non-adjacent lane would be a better choice (e.g. car is on lane 1 and planner only evaluates lanes 1 and 2, while lane 
3 on the far right may be free and therefore a good candidate lane to move towards). This would require more 
sophisticated path evaluation method where the planner assesses all lanes and decides ultimately which adjacent lane to
move to, based on the fact that the lane next to our adjacent lane will become a viable candidate to move to one the 
vehicle has reached the adjacent lane. The other concern we have with this approach is related to safety, since it is
more dangerous and tricky to execute such a bold move, because of the lateral distance that must be covered as well as
the difficulty of predicting the behaviour of other vehicles on the road.

Another improvement we should look into is to employ statistical techniques to better predict other vehicles' actions,
particularly anticipating when they merge into our lane as this increases the risk of a fatal collision. We could have
started off by using Naive Bayes but did not have enough time to dedicate to  testing and then picking the most 
discriminative features for our predictor.


Finally, our current planner only generates a single trajectory for a given possible next state, which means that we 
could be ignoring better trajectories for the same future state (e.g. further away/behind, or more on the left or 
right of a given lane). We have an implementation for such a multiple trajectories per state scheme, which assumes
that all final Frenet positions _s_ and _d_  (obtained by computing a jerk minial trajectory) follow a gaussian 
distribution with given means and standard deviation (mean_s, std_s) and (mean_d, std_d) for G(s) and G(d) 
distributions respectively. We must however choose the appropriate values for standard deviations, while the means 
will remain fixed at the originally desired end positions _end_s_  and _end_d_.


# Acknowledgements

This was so far without a doubt the most difficult project I have undertaken as part of this Self-Driving Car 
nanodegree, across all threee terms. I was not sure I could ever complete it and did not even know where to start
exactly! The small tutorial video from David Silver and Aaron Brown helped me to get started. Since I chose a jerk 
minimisation approach to trajectory generation, I did not find as many blog posts or references from students who
opted for this technique, as most articles were about using the spline technique in the tutorial from David and Aaron. 
But the paper from Werling and Kammel on [Optimal Trajectory Generation For Dynamic Street Scenarios in a Frenet Frame](http://video.udacity-data.com.s3.amazonaws.com/topher/2017/July/595fd482_werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame/werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame.pdf) really helped improve my 
intuition on how to use this topic.
I particularly enjoyed and felt inspired by Mithi's [articles](https://medium.com/@mithi/reflections-on-designing-a-virtual-highway-path-planner-part-2-3-392bc6cf11e7) on this 
project, as well as posts by other students.
Lastly, I am grateful to all the team at Udacity at 
their partners at Daimler to have put together such a 
great content and challenging project!