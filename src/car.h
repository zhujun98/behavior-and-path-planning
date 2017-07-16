//
// Created by jun on 7/16/17.
//

#ifndef BEHAVIOR_PLANNER_CAR_H
#define BEHAVIOR_PLANNER_CAR_H

#include <iostream>
#include <vector>
#include <map>
#include <string>

#include "traffic.h"

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
enum CarStates {KL, LCL, LCR, PLCL, PLCR};


class Car {

private:
  typedef std::map<int, std::vector<std::vector<int>>> state_pred;

  double px_;
  double py_;
  double vx_;
  double vy_;
  int lane_id_;

  double max_velocity_;
  double max_acceleration_;
  double max_steering_;

  double target_velocity_;

  CarStates state_;

  state_pred predictions_;

  Traffic traffic_;

  //
  // Update the vehicle's state
  //
  void update_state();

  //
  // Realize the planned state
  //
  void realize_state();

  void realize_keep_lane();

  void realize_lane_change(std::string direction);

  void realize_prep_lane_change(std::string direction);

public:

  //
  // constructor
  //
  Car();

  //
  // destructor
  //
  virtual ~Car();

  //
  // Move to the next time
  //
  void advance();
};


#endif //BEHAVIOR_PLANNER_CAR_H
