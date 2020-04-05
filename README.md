# Path Planning for Autonomous Highway Driving
Udacity Self-Driving Car Engineer Nanodegree Program

In this project our goal is to safely navigate around a virtual highway with other traffic that is driving +/-10 MPH of the 50 MPH speed limit. We are provided the car's localization and sensor fusion data, and a sparse map list of waypoints around the highway. "Our" ego car needs to travel as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible - but other cars will try to change lanes too. The ego car must avoid hitting other cars at all cost, and drive within the marked road lanes unless switching from one lane to another. The car must make at least one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Total acceleration must remain within 10 m/s^2 and jerk within 10 m/s^3. The main focus of this project is to elegantly combine current state and incoming information into a smooth path for the car to follow.

[//]: # (Image References)
[image1]: Progress1.jpg "Runtime Example"

## Basic Build Instructions

Running the code requires connecting to the Udacity CarND Term 3 Simulator, which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

This project requires [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by executing in the Linux bash:

```
$ git clone https://github.com/briansfma/CarND-Path-Planning-Project
$ cd CarND-Path-Planning-Project
$ mkdir build
$ cd build
$ cmake ..
$ make
$ ./path_planning
```

## Running the Program

If it is not running already, launch the project.

```
$ cd CarND-Path-Planning-Project
$ cd build
$ ./path_planning
```

When `path_planning` has initialized successfully, it will output to terminal

```
Listening to port 4567
```

Launch the simulator `term3_sim.exe`. Select "Project 1: Path Planning" from the menu. Upon successful connection to the simulator, `path_planning` will output to terminal

```
Connected!!!
```

The simulator will automatically start.

## Details

The ego car uses the following model to continuously decide its next steps of travel:

1. Take in sensor fusion data.

2. Make sense of the objects reported by sensor fusion. Prioritize the positions and speeds of only the closest objects ahead of us and the closest objects behind/beside us. We ignore objects that are too far (>150m) ahead of us or behind us, as the situation could change long before we get close to these objects.

3. Calculate what lane the car currently occupies. Default the lane choice here.

4. Calculate our proximity to anything immediately in front of us, and adjust speed accordingly to avoid collision. If there is nothing ahead of us, we default to the fastest legal speed possible.

5. If objects ahead of us are within "caring distance" (<40m ahead of us), we will weigh the desirability of each lane based on the position and speed of the objects ahead of us in each lane. The weight is calculated by three factors:
    - A linear combination of 1.5X the object's speed and 1X the object's distance ahead
    - If the lane is blocked by an object beside us, the weight is divided by 50X
    - If the lane is the ego car's current lane, the weight is boosted 5%

6. The lane with the largest relative weight is chosen as the target lane choice. However, if the optimal lane is more than 1 lane away from the ego car's current lane, we will only mark the next lane over as the target, in order to not make risky/jerky movements.

7. If a lane change is signaled, we must assess the gaps in traffic to position the car away from any awkward situations. Four logical options were identified that the ego car can choose from in order to make, or set itself up for, the lane change.
    - If the obstruction in the current lane is far enough ahead of an obstruction in the next lane, AND there is enough space behind the obstruction in the next lane, then the ego car will take the lane change early to be safe.
    - If the obstruction in the current lane is far enough ahead of an obstruction in the next lane, BUT there is NOT enough space behind the obstruction in the next lane, then the ego car will "shoot the gap", waiting until it passes the obstruction in the next lane before changing lanes.
    - If the obstruction in the current lane is NOT far enough ahead of an obstruction in the next lane, AND there is NOT enough space behind the obstruction in the next lane, then the ego car will slow down until there is enough space to change lanes behind the obstruction in the next lane.
    - If the obstruction in the current lane is NOT far enough ahead of an obstruction in the next lane, BUT there is enough space behind the obstruction in the next lane, then the ego car will take the lane change.

8. Lastly, before finalizing its decision, the ego car will check one more time that changing lanes will not run itself into an object in the target lane. If there is an obstruction still, the ego car will abort its lane change.

9. With the decision (required speed, lane keep/switch) made, we adjust the target speed based on what the ego car can actually achieve without breaking acceleration limits or physics.

10. We use the `spline.h` library ([link](http://kluge.in-chemnitz.de/opensource/spline/)) to draw smooth path lines for the ego car to follow. The spline is drawn with 5 anchor points:
    - The upcoming two points from the car's last path it was following (if this doesn't exist, we generate substitutes using the car's position and heading)
    - Three points in the car's target lane, 40m, 75m, and 90m ahead of the car's current position

11. Based on the target speed of the car, path points are calculated evenly along the path spline, and packaged into a JSON object and sent to the simulator.


## Expected Behavior/Known Bugs

The implementation in this project is believed to work well (the car is aggressive, but safe making lane changes in order to maximize speed without causing incidents), easily driving for an hour straight, navigating tricky traffic jams, etc. without incident. HOWEVER, at a certain location along the highway lap, the Term3 Simulator sometimes slows down, and when this occurs, the "Violated Speed Limit!" error can randomly occur despite the ego car never traveling above 50 MPH. This seems to be avoidable (at least, on the machine this project was written on) by manually setting the camera view in the simulator to overhead. As long as the simulator does not slow down, one could expect the ego car to perform like such:

![alt text][image1]

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



