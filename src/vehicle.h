//
// Created by jun on 7/16/17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <iostream>
#include <vector>
#include <list>


class Map;


class Vehicle {

protected:

  bool is_initialized_;

  double time_step_;

  // parameters in global coordinate system
  double px_; // in m
  double py_; // in m
  double speed_; // in m/s
  double yaw_; // in m/s
  // parameters in Frenet-Serret coordinate system
  double ps_; // in m
  double pd_; // in m
  double vs_; // in m/s
  double vd_; // in m/s
  double as_; // in m/s^2
  double ad_; // in m/s^2

  unsigned history_length_; // length of historic data
  std::list<double> hps_;
  std::list<double> hpd_;
  std::list<double> hvs_;
  std::list<double> hvd_;

  const Map* map_;

  int lane_id_;

public:

  //
  // constructor
  //
  Vehicle(const Map& map);

  //
  // destructor
  //
  virtual ~Vehicle();

  //
  //
  //
  void updateVelocityHistory(const std::list<double>& p, std::list<double>& v);

  //
  // Update the vehicle's state
  //
  virtual void updateParameters(const std::vector<double>& localization);

  //
  // Print out the vehicle's state
  //
  void printout() const;

  double getTimeStep() const;

  int getLaneID() const;

  double getPx() const;

  double getPy() const;

  double getSpeed() const;

  double getYaw() const;

  double getPs() const;

  double getPd() const;

  double getVs() const;

  double getVd() const;

  double getAs() const;

  double getAd() const;
};

#endif //PATH_PLANNING_VEHICLE_H
