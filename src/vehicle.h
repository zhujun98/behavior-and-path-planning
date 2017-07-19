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
enum VehicleStates {KL, LCL, LCR, PLCL, PLCR};

typedef std::map<int, std::vector<std::vector<int>>> state_pred;
typedef std::pair<std::vector<double>, std::vector<double>> vehicle_traj;

/*
 * Vehicle base class (abstract class)
 */

class VehicleBase {

protected:
  // whether the object has been initialized
  bool is_initialized_;

  // current lane id
  int lane_id_;

  // current positions in global coordinate system
  double px_; // m
  double py_; // m
  // current positions in Frenet-Serret coordinate system
  double ps_;  // m
  double pd_;  // normalized to 1

  double speed_;  // current speed (m/s)
  double acceleration_;  // current acceleration (m/s^2)
  double yaw_;  // current yaw angle (rad) in global Cartesian coordinate system

  double max_speed_;  // maximum speed (m/s)
  double max_acceleration_;  // maximum acceleration (m/s^2)
  double max_steering_;  // maximum steering angle (rad)

  //
  // constructor
  //
  VehicleBase();

public:

  //
  // destructor
  //
  virtual ~VehicleBase();

  virtual void printout() = 0;
};


/*
 * Other cars on the road
 */

class Car : public VehicleBase {

public:
  //
  // constructor
  //
  Car();

  //
  // destructor
  //
  ~Car();

  void update_state(const std::vector<double>& localization);

  void printout();
};

/*
 *
 */

class Ego : public VehicleBase {

private:

  std::vector<double> path_s_;
  std::vector<double> path_d_;

  int predicted_points_;  // No. of points to predict

  double time_step_;  //

  VehicleStates state_;

  //
  //
  //
  void keep_lane();

  //
  //
  //
  void change_lane(std::string direction);

  //
  //
  //
  void prep_lane_change(std::string direction);

public:

  //
  // constructor
  //
  Ego();

  //
  // destructor
  //
  virtual ~Ego();

  //
  // Update the vehicle's state
  //
  void update_state(const std::vector<double>& localization,
                    const vehicle_traj& continued_path);

  //
  // Realize the planned state
  //
  vehicle_traj plan_path();

  //
  //
  //
  void printout();
};


#endif //PATH_PLANNING_VEHICLE_H
