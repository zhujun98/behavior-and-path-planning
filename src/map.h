//
// Created by jun on 7/17/17.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <iostream>
#include <vector>


typedef std::pair<std::vector<double>, std::vector<double>> vehicle_traj;


class Map {

private:

  int n_lanes_; // number of lanes

  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> s_;
  std::vector<double> dx_;
  std::vector<double> dy_;

  // The max s value before wrapping around the track back to 0
  double max_s_;

  //
  // Read way points data from a file
  //
  void load_data();

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
  //
  //
  int closestWaypoint(double x, double y) const;
  int nextWaypoint(double x, double y, double theta);
  //
  //
  //
  std::pair<double, double> cartesianToFrenet(double x, double y) const;

  //
  //
  //
  std::pair<double, double> frenetToCartesian(double s, double d) const;

  //
  //
  //
  vehicle_traj trajFrenetToCartesian(const vehicle_traj& traj_in_cartesian);

  vehicle_traj trajCartesianToFrenet(const vehicle_traj& traj_in_cartesian);
};

#endif //PATH_PLANNING_MAP_H
