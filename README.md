# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.


### Discussion
This simple implementation is quite bare bones, yet is able to complete more than 5 miles without incident

The car drives according to the speed limit of 50MPH using simple logic in lines ~455:

    } else if(ref_vel < 49){
      ref_vel += 0.7;

The 0.7 value was chosen as to not exceed man jerk and acceleration

The car does not have collisions because it changes lanes when getting to a "too close" state. Too close is defined in lines ~408:

    if(headway <= 1.3){
      too_close = true;
Where headway (with units of seconds) is the distance to forward vehicle within the same lane (in meters) devided by the ego vehicle's speed (in meters per second)

The lane changes occur according to a cost function scheme.  Lines ~311 define the 3 costs:

    double left_cost = 0;
    double stay_cost = -0.01;
    double right_cost = 0;

These are the costs to change to the left/right lane, relative to the current lane, or stay in the current lane.  The initial value of stay cost below zero gives the car a slight preference to stay in the current lane. When the "too close" flag is on, the car will choose the lane according to the lowest cost. Note that speed could have also been modulating using this cost scheme, but the simpler if/then logic of lines ~427 and ~455 worked well enough.

For each cost, first a determination was made to calculate if the other vehicle is in the left, current, or right lane relative to the current lane. The logic for left lane is in line ~354:

    // Rules for cars on the left
    if(d < (4*lane) && d> (4*(lane-1))){

Here, lane is the current lane.  4 is the width of each lane in meters.

The appropriate cost if then formulated using an expoential decay scheme in line ~358:

    cost = gap_weight*exp(-headway/front_char_hw);
    
The headway is normalized by a "characteristic headway" and the exponent is weighted by a gap weight.

Lines ~436 show the logic for choosing the lane associated with the minimum cost. A lane change is declared using the ChangeLane helper function, which returns the number of the new lane (0 is left lane, 1 is middle, 2 is right lane).  ChangeLane can also reject the lane change request and return the current lane. 

A smooth lane change is then implemented using the sparsely spaced waypoints scheme in lines ~521. These are then connected using a spline in lines ~579.  This comes from the class walkthrough lessons.

---   
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

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


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



