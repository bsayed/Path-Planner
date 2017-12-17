# CarND-Path-Planning-Project


### Goals
In this project the goal is to safely navigate around a virtual highway with other
traffic that is driving +-10 MPH of the 50 MPH speed limit. 
You will be provided the car's localization and sensor fusion data,
there is also a sparse map list of waypoints around the highway.
The car should try to go as close as possible to the 50 MPH speed limit,
which means passing slower traffic when possible, note that other cars will try to change lanes too. 
The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, 
unless going from one lane to another. 
The car should be able to make one complete loop around the 6946m highway. 
Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. 
Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.


### Trajectory Generator
The trajectory generator uses a Cubic Spline interpolation library that is implemented as a
single C++ header file "spline.h" which can be found at this [link](http://kluge.in-chemnitz.de/opensource/spline/). Initially we get the previous and the current position 
from the path tangent of the car then we add 3 more waypoints that are 30, 60, and 90 meters
away from the current car position "s" in the Fernet coordinate system. The 3 waypoints then are converted
to the cartesian coordinate system and combined with the first 2 points to form a group of 5 waypoints
that are converted to the vehicle coordinate system then get feed to the spline interpolation library.
The resulted spline is used to calculate how to break the trajectory onto equally spaced points that will 
help the car travel at the desired velocity. Initially the trajectory generator generates 50 waypoints to
guide the Ego car on the desired speed. The waypoints are converted to the Map
coordinate system before passing them back to the simulator. In the subsequent trajectory generation, we copy
the waypoints from the previous path supplied by the simulator and we use the last 2 points 
as the starting points then we add the 3 waypoints that are 30, 60, and 90 meters away
in the same way we did in the initial trajectory generation. We still end up with 50 waypoints however, 
this time we only interpolate the difference between the previous path and the needed 50 points.  

The trajectory generator uses a behavior-planner that is invoked when there is a car
in front of the Ego car in the same lane. Before invoking the behavior-planner, we check to see
if it is safe to change to the adjacent lanes or not by looking at the distance between the Ego car
and the cars in the adjacent lanes. We depend on the sensor fusion data to achieve that.
The behavior planner supports 3 states, keep lane to stay in the same lane when it is not safe to change lane, lane change 
right to transition from the current lane to the lane to the right side, and lane change
left to transition to the lane on the left side. As an optimization, the behavior-planner is only invoked 
when we have 3 choices, if we only end up with 2 choices, which means that one of them is always keep lane
we choose the other choice which will be LCR or LCL, the justification is to reach our goal faster if we
change the lane rather than keep our lane behind a slower car. The implementation of the behavior-planner
follows the one introduced in the case plus an adaptation of a python project hosted at this
[link](https://d17h27t6h515a5.cloudfront.net/topher/2017/September/59b45364_behavior-planner-python-3/behavior-planner-python-3.zip).
 