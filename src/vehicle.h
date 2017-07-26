//
// Created by jun on 7/16/17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <iostream>
#include <vector>


class Map;


class Vehicle {

protected:

  bool is_initialized_;

  // parameters in global coordinate system
  double px_; // in m
  double py_; // in m
  double vx_; // in m/s
  double vy_; // in m/s
  // parameters in Frenet-Serret coordinate system
  double ps_; // in m
  double pd_; // in m

  int lane_id_;

public:

  //
  // constructor
  //
  Vehicle();

  //
  // destructor
  //
  virtual ~Vehicle();

  //
  // Update the vehicle's state
  //
  virtual void updateParameters(const std::vector<double>& localization,
                                const Map& map);

  //
  // Print out the vehicle's state
  //
  void printout() const;

  int getLaneID() const;

  double getPx() const;

  double getPy() const;

  double getVx() const;

  double getVy() const;

  double getPs() const;

  double getPd() const;
};

#endif //PATH_PLANNING_VEHICLE_H
