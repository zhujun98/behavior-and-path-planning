//
// Created by jun on 7/17/17.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <iostream>
#include <vector>


typedef std::pair<std::vector<double>, std::vector<double>> vehicle_trajectory;
typedef std::vector<std::vector<double>> local_waypoints;


class Map {

private:

  int n_lanes_; // number of lanes
  int lane_width_; // width of lane (m)
  double max_s_; // The max s value before wrapping around the track back to 0

  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> s_;
  std::vector<double> dx_;
  std::vector<double> dy_;

  //
  // Read way points data from a file
  //
  void loadData();

  long i_min_x_; // way point index having the min x value
  long i_max_x_; // way point index having the max x value
  long i_min_y_; // way point index having the min y value
  long i_max_y_; // way point index having the max y value

public:

  //
  // constructor
  //
  Map();

  //
  // destructor
  //
  ~Map();

  //
  // Computer the lane id using Frenet d value
  //
  int compute_lane_id(double d) const;

  //
  // Calculate the index of the closest waypoint to (x, y)
  //
  int closestWaypoint(double x, double y) const;

  //
  //
  //
  int nextWaypoint(double x, double y, double theta) const;

  //
  // Convert a point from Frenet to Cartesian coordinate system
  //
  std::pair<double, double> frenetToCartesian(double s, double d) const;

  //
  // Convert a point from Cartesian to Frenet coordinate system
  //
  std::pair<double, double> cartesianToFrenet(double x, double y) const;

  //
  // Convert a trajectory from Frenet to Cartesian coordinate system
  //
  vehicle_trajectory trajFrenetToCartesian(const vehicle_trajectory& trajectory) const;

  //
  // Convert a trajectory from Cartesian to Frenet coordinate system
  //
  vehicle_trajectory trajCartesianToFrenet(const vehicle_trajectory& trajectory) const;

  double getLaneWidth() const;

  double getMaxS() const;
};

#endif //PATH_PLANNING_MAP_H
