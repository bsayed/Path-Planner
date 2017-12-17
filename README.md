# CarND-Path-Planning-Project
   
### Goals
In this project the goal is to implement a path-planner that is capable of safely navigating a car around a virtual highway with other
traffic that is driving +-10 MPH of the 50 MPH speed limit. 
The path-planner is provided with the car's localization and sensor fusion data from the simulator,
there is also a sparse map list of waypoints around the highway.
The path-planner was able to bring the car speed as close as possible to the 50 MPH speed limit,
which means passing slower traffic when possible. 
The path-planner was able to avoid hitting other cars as well as driving inside of the marked road lanes at all times, 
unless going from one lane to another. 
The path-planner was be able to make one complete loop around the 6946m highway successfully. 
The car did not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3 at any given time.


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
 

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).


#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

1. ["x"] The car's x position in map coordinates

1. ["y"] The car's y position in map coordinates

1. ["s"] The car's s position in frenet coordinates

1. ["d"] The car's d position in frenet coordinates

1. ["yaw"] The car's yaw angle in the map

1. ["speed"] The car's speed in MPH

#### Previous path data given to the Planner

1. ["previous_path_x"] The previous list of x points previously given to the simulator

1. ["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

1. ["end_path_s"] The previous list's last point's frenet s value

1. ["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

1. ["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

