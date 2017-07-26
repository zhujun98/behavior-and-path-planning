//
// Created by jun on 7/16/17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <iostream>
#include <vector>


class Vehicle {

  friend class PathPlanner;

protected:

  // current positions in global coordinate system
  double px_; // m
  double py_; // m
  double vx_; // m/s
  double vy_; // m/s
  // current positions in Frenet-Serret coordinate system
  double ps_;  // m
  double pd_;  // m

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
  virtual void update(const std::vector<double>& localization);

  //
  // Print out the vehicle's state
  //
  void printout() const;

  int getLaneID() const;

  void setLaneID(int value);

  double getVx() const;
  double getVy() const;
  double getPs() const;
  double getPd() const;
};

#endif //PATH_PLANNING_VEHICLE_H
