# Behavior and Path Planning
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
[![Build Status](https://travis-ci.com/zhujun98/behavior-and-path-planning.svg?branch=master)](https://travis-ci.com/zhujun98/behavior-and-path-planning)


In this project, the goal is to safely navigate around a virtual highway with 
other vehicles that are driving +-10 MPH of the 50 MPH speed limit. You will be 
provided the car's localization and sensor fusion data, there is also a sparse 
map list of waypoints around the highway. The car should try to go as close as 
possible to the 50 MPH speed limit, which means passing slower traffic when 
possible, note that other cars will try to change lanes too. The car should 
avoid hitting other cars at all cost as well as driving inside of the marked 
road lanes at all times, unless going from one lane to another. The car should 
be able to make one complete loop around the 6946 m highway. Since the car 
is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 
loop. Also the car should not experience total acceleration over 10 m/s^2 and 
jerk over 10 m/s^3.

## Dependencies

- [uWebSockets](https://github.com/uNetworking/uWebSockets) **0.14.8**
- [Eigen3](https://eigen.tuxfamily.org/dox/) **3.3.7**

## Download

```sh
$ git clone https://github.com/zhujun98/behavior-and-path-planning.git
```

Update submodules ([Eigen3](https://eigen.tuxfamily.org/dox/))

```sh
$ git submodule init
$ git submodule update
```

## Build, install and run

- Download the [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)

- Install [uWebSockets](https://github.com/uNetworking/uWebSockets)
```sh
$ ./scripts/install_uWS_ubuntu.sh
```

### With [CMake](https://cmake.org/)

```sh
$ mkdir build && cd build
$ cmake .. && make -j4
$ cd apps
$ ./run_app MAP_FILE # the default MAP_FILE is "data/highway_map.csv"
```

To use [boost.log](https://www.boost.org/doc/libs/1_67_0/libs/log/doc/html/index.html) in this project:

```sh
$ ./scripts/install_boost.sh
$ mkdir build && cd build
$ cmake -DWITH_BOOST .. && make -j4
```

To build and run the test:

```shell script
$ mkdir build && cd build
$ cmake -DBUILD_TESTS .. && make -j4
$ make unittest
```

### With [Bazel](https://bazel.build/)

Install [Bazel](https://bazel.build/) following the [official instruction](https://docs.bazel.build/versions/master/install-ubuntu.html#installing-menu) and then type

```
$ bazel build ...
$ bazel run apps:run_app MAP_FILE
```

## Highway map
The map data of the highway is listed in [highway_map.csv](data/highway_map.csv). Each row of the data contains  [x, y, s, dx, dy] values for a waypoint, where x and y are the coordinates in the global coordinate system, s is the longitudinal coordinate along the reference trajectory, dx and dy define the x and y components of the unit vector d which is normal (pointing to the right of the traffic direction) to the reference trajectory.

![highway map](./data/highway_map.png)

## Simulator output

* Localization data (without noise)
  - ["x"] x in the Cartesian coordinate system, m
  - ["y"] y in the Cartesian coordinate system, m
  - ["s"] s in the Frenet coordinate system, m
  - ["d"] d in the Frenet coordinate system, m
  - ["yaw"] yaw angle, degree
  - ["speed"] speed, MPH

* Sensor fusion data  (without noise)

  - ["sensor_fusion"] A list other cars' data on the same side of the road in 
  the format [[ID, x (m), y (m), vx (m/s), vy (m/s), s (m), d (m)]]

* Unprocessed previous path data passed to the simulator (not used)

  - ["previous_path_x"] lists of unprocessed x coordinates, m
  - ["previous_path_y"] lists of unprocessed y coordinates, m

* End point of the previous path (not used) 

  - ["end_path_s"] s in the Frenet coordinate system, m
  - ["end_path_d"] d in the Frenet coordinate system, m

## Behavior planning

Behavior planning is achieved by using the FSM pattern:

**Start Up** -> **Keep Lane** <-> **Change Lane** 


## Path planning

The following strategy is applied in searching the optimized path:

- Estimate the feasible final dynamics (s, d, speed, etc.) of the car in a given
state;
- Greedily search the [jerk minimizing trajectories](http://ieeexplore.ieee.org/document/5509799/) 
which takes the shortest time and has the shortest distance;
- Collision check is applied in the **Change Lane** state. If a collision could
happen for the planned path, the car will keep going in the current lane. If 
a valid path can not be found after a certain time, it will switch back to the
**Keep Lane** state.


## Videos (to be updated...)

[![alt text](http://img.youtube.com/vi/lbwL3iqhXzE/0.jpg)](https://youtu.be/lbwL3iqhXzE)

Increase the speed to 85 MPH. More exciting!

[![alt text](http://img.youtube.com/vi/7MIDTK7BHy4/0.jpg)](https://youtu.be/7MIDTK7BHy4)





