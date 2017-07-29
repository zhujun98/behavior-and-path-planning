//
// Created by jun on 7/17/17.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <iostream>
#include <vector>


typedef std::pair<std::vector<double>, std::vector<double>> vehicle_trajectory;


class Map {

private:

  size_t n_lanes_; // number of lanes
  int lane_width_; // width of lane (m)
  double max_s_; // The max s value before wrapping around the track back to 0

  // Coordinates of the way points
  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> s_;
  std::vector<double> dx_;
  std::vector<double> dy_;

  // Read way points data from a file
  void loadData();

public:

  // constructor
  Map();

  // destructor
  ~Map();

  // Computer the lane id using Frenet d value
  int computerLaneID(double d) const;

  // Calculate the index of the closest waypoint to (x, y)
  int closestWaypoint(double x, double y) const;

  //
  int nextWaypoint(double x, double y, double theta) const;

  // Convert a point from Frenet to Cartesian coordinate system
  std::pair<double, double> frenetToCartesian(double s, double d) const;

  // Convert a point from Cartesian to Frenet coordinate system
  std::pair<double, double> cartesianToFrenet(double x, double y) const;

  // Convert a trajectory from Frenet to Cartesian coordinate system
  vehicle_trajectory trajFrenetToCartesian(const vehicle_trajectory& trajectory) const;

  // Convert a trajectory from Cartesian to Frenet coordinate system
  vehicle_trajectory trajCartesianToFrenet(const vehicle_trajectory& trajectory) const;

  double getLaneWidth() const;

  double getMaxS() const;

  size_t getNoLanes() const;
};

#endif //PATH_PLANNING_MAP_H
