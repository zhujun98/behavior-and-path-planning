//
// Created by jun on 7/16/17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <iostream>
#include <vector>
#include <map>
#include <string>

#include "traffic.h"
#include "path_planner.h"
#include "map.h"

//
// "KL" - Keep Lane
// - The vehicle will attempt to drive at its target velocity, unless
//  there is traffic in front of it, in which case it will slow down.
//
//  "LCL" or "LCR" - Lane Change Left / Right
//  - The vehicle will start to change lanes and then follow longitudinal
//    behavior for the "KL" state in the new lane.
//
// "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
//   - The vehicle will find the nearest vehicle in the adjacent lane
//     which is BEHIND itself and will adjust speed to try to get behind
//     that vehicle.
//
enum VehicleBehavior {KL, LCL, LCR, PLCL, PLCR};

typedef std::map<int, std::vector<std::vector<int>>> state_pred;
typedef std::pair<std::vector<double>, std::vector<double>> vehicle_traj;

//class PathPlanner;

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

  std::vector<double> path_s_;
  std::vector<double> path_d_;

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
  void update(const std::vector<double>& localization, const Map& map);

  //
  // Print out the vehicle's state
  //
  void printout() const;
};


/*
 * The ego car
 */

class Ego : public Vehicle {

  friend class PathPlanner;

private:
  VehicleBehavior behavior_;

  double max_speed_;  // maximum speed (m/s)
  double max_acceleration_;  // maximum acceleration (m/s^2)
  double max_steering_;  // maximum steering angle (rad)

public:

  //
  // constructor
  //
  Ego();

  //
  // destructor
  //
  virtual ~Ego();

  VehicleBehavior getBehavior();
  void setBehavior(VehicleBehavior value);

  vehicle_traj getPath();

};


#endif //PATH_PLANNING_VEHICLE_H
