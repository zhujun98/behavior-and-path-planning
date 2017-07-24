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
//  "LC"  - Lane Change
//  - The vehicle will start to change lanes and then follow longitudinal
//    behavior for the "KL" state in the new lane.
//
// "PLC" - Prepare for Lane Change
//   - The vehicle will find the nearest vehicle in the adjacent lane
//     which is BEHIND itself and will adjust speed to try to get behind
//     that vehicle.
//
enum VehicleBehavior {KL, LC, PLC};

typedef std::map<int, std::vector<std::vector<int>>> state_pred;
typedef std::pair<std::vector<double>, std::vector<double>> vehicle_traj;


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

  int getLaneID();

  void setLaneID(int value);

  double getPd();
};


/*
 * The ego car
 */

class Ego : public Vehicle {

private:
  VehicleBehavior behavior_;

  int target_lane_id_;
  double lane_change_timer_;

  double max_speed_;  // maximum speed (m/s)
  double max_acceleration_;  // maximum acceleration (m/s^2)
  double max_steering_;  // maximum steering angle (rad)

public:
  std::vector<double> path_s_;
  std::vector<double> path_d_;
  //
  // constructor
  //
  Ego();

  //
  // destructor
  //
  virtual ~Ego();

  //
  // Update the ego car's state
  //
  void update(const std::vector<double>& localization);

  //
  // Remove the way points in the vehicle paths which have been processed.
  //
  void truncatePath();

  VehicleBehavior getBehavior();

  void setBehavior(VehicleBehavior value);

  vehicle_traj getPath();

  int getTargetLaneID();

  void setTargetLaneID(int value);

  double getLaneChangeTimer();

  void setLaneChangeTimer(double value);

  double getMaxSpeed();

};


#endif //PATH_PLANNING_VEHICLE_H
