//
// Created by jun on 7/16/17.
//

#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

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
typedef std::map<int, std::vector<std::vector<int>>> state_pred;
typedef std::pair<std::vector<double>, std::vector<double>> car_traj;

class Car {

private:

  bool is_initialized_;

  std::vector<double> path_x_;  //
  std::vector<double> path_y_;  //
  int predicted_points_;

  double time_step_;  //
  double speed_;  //
  double acceleration_;  // m^2
  int lane_id_;


  double max_speed_;  // maximum speed (m/s)
  double max_acceleration_;  // maximum acceleration (m/s^2)
  double max_steering_;  // maximum steering angle (deg)
  double target_velocity_;

  CarStates state_;

  state_pred predictions_;

  Traffic traffic_;

  //
  // Update the vehicle's state
  //
  void update_state(car_traj continued_path);

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
  car_traj advance(car_traj continued_path,
               std::map<int, std::vector<double>> sensor_fusion);
};


#endif //PATH_PLANNING_CAR_H
