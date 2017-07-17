# Path Planning
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
   
Jun Zhu

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2  nd jerk that is greater than 50 m/s^3.

## Highway map
The map data of the highway is listed in [highway_map.csv](data/highway_map.csv). Each row of the data contains  [x, y, s, dx, dy] values for a waypoint, where x and y are the coordinates in the global coordinate system, s is the longitudinal coordinate along the reference trajectory, dx and dy define the x and y components of the unit vector d which is normal (pointing to the right of the traffic direction) to the referece trajectory.

## Simulator output

#### Main car's localization data (without noise)

["x"] / ["y"] / ["s"] / ["d"] Coordinates in the global coordinate system and Frenet coordinate system.

["yaw"] Yaw angle in degree.

["speed"] Speed in MPH.

Note: these data will not be used except for the initialization of the car's path.

#### Unreached previous path data passed to the planner

["previous_path_x"] / ["previous_path_y"] Lists of x and y values.

These two lists are very important for the project. At least part of these two lists should be kept as the beginning points of the new path data.

#### End s and d values of  the previous path 

["end_path_s"] / ["end_path_d"] The last s and d values.

#### Sensor fusion data  (without noise)

["sensor_fusion"] A list other cars' data on the same side of the road in the format [[ID, x (m), y (m), vx (m/s), vy (m/s), s (m), d]]

## Tips

**Forget about the physics / object dynamics!!!**

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.
