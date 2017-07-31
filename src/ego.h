//
// Created by jun on 7/25/17.
//

#ifndef PATH_PLANNING_EGO_H
#define PATH_PLANNING_EGO_H

#include <iostream>
#include <vector>

#include "vehicle.h"
#include "path_planner.h"

class EgoState;
class Map;


class Ego : public Vehicle {

private:
  EgoState* state_; // vehicle state

  typedef std::vector<std::vector<std::vector<double>>> Surroundings;
  Surroundings surroundings_; // surrounding vehicles information

  int target_lane_id_;
  double target_speed_;

  std::vector<double> path_s_;
  std::vector<double> path_d_;

  double max_speed_; // maximum speed (m/s)
  double max_acceleration_; // maximum acceleration (m/s^2)
  double max_jerk_; // maximum jerk (m/s^3)
  double max_steering_; // maximum steering angle (rad)
  double min_safe_distance_;

  // The actual evaluation distance is given by the minimum of
  // max_evaluation_time_* speed_ and max_evaluation_distance_
  //
  // We care about traffic within this distance (m). Also, in reality,
  // the detector has its own range.
  double max_evaluation_time_;
  double max_evaluation_distance_;

  long ticker_;  // a timer controlling output

public:

  // constructor
  Ego(const Map& map);

  // destructor
  virtual ~Ego();

  // Update the ego car's state
  void update(const std::vector<double>& localization,
              const std::vector<std::vector<double>>& sensor_fusion);

  // Remove the way points in the vehicle paths which have been processed.
  void updateUnprocessedPath();

  // Update information of the surrounding vehicles
  void updateSurroundings(const std::vector<std::vector<double>>& sensor_fusion);

  //
  // Get the closest vehicle (in front of or in back of) on a given lane
  //
  // @param lane_id: lane ID
  // @param direction: 1 for front, -1 for rear
  // @return: [s, v, d] of the vehicle
  //
  std::vector<double> getClosestVehicle(int lane_id, int direction) const;

  //
  // Keep a given number of way points in the current path
  //
  // @param n_keep: number of points to keep
  //
  void truncatePath(unsigned int n_keep);

  //
  // Extend the current path to a given duration
  //
  void extendPath(vehicle_trajectory new_path);

  // Print out the vehicle information around the ego car
  void printTraffic();

  std::vector<double> const* getPathS() const;

  std::vector<double> const* getPathD() const;

  double getMaxSpeed() const;

  double getMaxAcceleration() const;

  double getMaxJerk() const;

  double getMaxSteering() const;

  double getMinSafeDistance() const;

  double getMaxEvaluationDistance() const;

  Surroundings const* getSurroundings() const;

  Map const* getMap() const;

  int getTargetLaneID() const;

  void setTargetLaneID(int value);

  double getTargetSpeed() const;

  void setTargetSpeed(double value);

  long getTicker() const;
};


#endif //PATH_PLANNING_EGO_H
