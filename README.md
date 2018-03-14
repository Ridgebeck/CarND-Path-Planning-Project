# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Project Writeup
Here I describe briefly what the objective of the project was and in a bit more detail how my solution to the problem looks like.

#### File Structure
The program is written in C++ and interacts with the Udacity Term 3 Simulator via WebSocket. The general structure is as follows:
* the program receives data from the simulator via WebSocket
* the program calculates a path based on that data and the map data from another source
* the program sends the generated path points back to the simulator, which then simulates the driving behavior of the car

The important files are:
* *main.cpp* contains all of the necessary code for path generation
* *spline.h* contains additional functions to generate spline curves and is included in main.cpp
* map data is contained in the folder data in the CSV file *highway_map.csv*

#### General Approach
* data of the car as well as other cars on the highway (sensor fusion data) is read from the simulator
* general safety zones in front and to the side of the car are defined
* distance to closest obstacles in safety zones are saved and flag is set (including predicted future states)
* safety zones to the side are modified based on the obstacle (slower car) in the front of our car
* car speed is adjusted based on the speed and distance of the obstacle in front
* decision for a lane change is made based on the obstacle data in all safety zones
* old path data is recognized and the last points of the previous path are taken as the start of the new one
* a new path is generated based on the decision to keep or switch lanes via a spline curve
* spline curve gets interpolated based on the target speed of th car
* old and new path points are combined to one path
* path points are sent back to the simulator

#### Path Generation in Detail

The decision for the lane change was made on the following logic:

* Is there an obstacle (slower car) in front of the vehicle?
* Depending on the car's current lane - are the adjacent lanes save for a lane change?
* If both coditions are true, change the lane. If it is not possible, adjust the speed to follow the car in front in a safe distance.

The path generation was conducted with the help of a spline function in order to smoothen the vehicles path and avoid a jerky motion profile. During my test, I noticed a balancing problem between accurately following the lane in the center and a smooth path between the waypoints. The most important factor was the distance between the spline points - a smaller distance generates a more accurate path, a larger distance a less jerky and smoother one. After a few test rund with different values I decided to go with two different values - 35m holding the lane and 55m during a lane change. That could further be optimized to make it dependend on the speed of the car.

Instead of generating a path from scratch every time the algorithm considers the previously generated path points. The spline function takes the last two points from the previously generated path as the starting points. In the very first cycle when no previous path data is available, the current car position and a previous position (estimated backwards by current position and current angle of the car).

After that I took the current car position, added the value for distance in s (35 or 55m), combined it with the target lane (as d value) and converted it over to X and Y coordinates with the help of the function *getXY()*. I then applied a simple linearization in order to interpolate the spline. As the car moves to the next point every 20ms, the distance between the points had to be calculated based on the target speed. This was achieved by taking the target speed and divide it by the number of steps (total time divided by update time).

After that the distance between the car and the next point on the spline was taken and divided by the calculated distance to get the number of increments for that distance. The distance on the X axis was then divided by this number to gee a good estimation for the increment distance on the X axis and therefore the interpolated X values on the spline. Based on the amount of previous path points the X and the corresponding Y values were then added to the path and the complete path was given to the simulator to execute the motion.

#### Performance
With some tweaking of the parameters for distance, speed, etc. I was able to get the car to drive around the course for over 1 hour consistently. The algorithm could be further optimized to adjust the path during a lane change according to the speed and the distance to the car in front. Furthermore, the car could take the data of the cars in fornt of it into account to evaluate all lanes and target the one with the fastest or no traffic (lowest cost). This is especially noticeable with the current implementation when the car is  on one of the outside lanes and it doesn't consider anything but the adjacent lane or when the car is in the middle lane and has the choice to go either left or right (right now it always changes to the left lane).

---------------------------------------------------------------------------------------------------------------------

### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

