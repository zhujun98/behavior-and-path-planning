# Behavior and path Planning
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
   
Jun Zhu

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2  and jerk over 10 m/s^3.

## Setup and run

Download the [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2) which contains the Path Planning Project.

Run the simulator and 

```
$ mkdir build && cd build
$ cmake .. && make
$ ./path_planning
```


## Highway map
The map data of the highway is listed in [highway_map.csv](data/highway_map.csv). Each row of the data contains  [x, y, s, dx, dy] values for a waypoint, where x and y are the coordinates in the global coordinate system, s is the longitudinal coordinate along the reference trajectory, dx and dy define the x and y components of the unit vector d which is normal (pointing to the right of the traffic direction) to the reference trajectory.

## Simulator output

* Ego car's localization data (without noise)
  - ["x"] x in the Cartesian coordinate system, m
  - ["y"] y in the Cartesian coordinate system, m
  - ["s"] s in the Frenet coordinate system, m
  - ["d"] d in the Frenet coordinate system, m
  - ["yaw"] yaw angle, degree
  - ["speed"] speed, MPH

* Unprocessed previous path data passed to the simulator (not used)

  - ["previous_path_x"] lists of unprocessed x coordinates, m
  - ["previous_path_y"] lists of unprocessed y coordinates, m

* End point of the previous path (not used) 

  - ["end_path_s"] s in the Frenet coordinate system, m
  - ["end_path_d"] d in the Frenet coordinate system, m

* Sensor fusion data  (without noise)

  - ["sensor_fusion"] A list other cars' data on the same side of the road in the format [[ID, x (m), y (m), vx (m/s), vy (m/s), s (m), d (m)]]

## Behavior planning

Behavior planning is achieved by using the FSM pattern. We have two types of FSMs: ***state*** (belong to the ego vehicle) and ***transition state*** (belong to the ***state***). The ego vehicle can only occupy one ***state*** at a time but the ***state*** can have several ***transition states***. At each time step, the ***transition states*** of the current ***state*** are checked one by one. If any transition is valid, the vehicle will switch to the next ***state*** that this ***transition state*** leads to.

## Path planning

Path planning can be divided into two steps: 
1. Estimate the final motion state (s, d, speed, etc.) of the ego vehicle based on the current motion state of the ego vehicle and the surrounding vehicles. Collision check is also done in this step.
2. Generate several [jerk minimizing trajectories](http://ieeexplore.ieee.org/document/5509799/) by jittering the the final motion state. Then a brute force search is applied to select the trajectory with the minimum cost (a function of average speed, maximum speed, maximum acceleration, maximum jerk, etc.).

## Videos

Solution for the project.

[![alt text](http://img.youtube.com/vi/lbwL3iqhXzE/0.jpg)](https://youtu.be/lbwL3iqhXzE)

Increase the speed to 85 MPH. More exciting!

[![alt text](http://img.youtube.com/vi/7MIDTK7BHy4/0.jpg)](https://youtu.be/7MIDTK7BHy4)





