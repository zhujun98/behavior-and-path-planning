//
// Created by jun on 7/25/17.
//

#ifndef PATH_PLANNING_EGO_H
#define PATH_PLANNING_EGO_H

#include <iostream>
#include <vector>

#include "vehicle.h"

class EgoState;
class Map;


class Ego : public Vehicle {

private:
  EgoState* state_;

  typedef std::vector<std::vector<std::vector<double>>> Surroundings;
  Surroundings surroundings_;

  int target_lane_id_;
  double target_speed_;

  std::vector<double> path_s_;
  std::vector<double> path_d_;

  double max_speed_;  // maximum speed (m/s)
  double max_acceleration_;  // maximum acceleration (m/s^2)
  double max_jerk_; // maximum jerk (m/s^3)
  double max_steering_;  // maximum steering angle (rad)
  double min_safe_distance_;

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
  // @param coeff_s: polynomial coefficents for s coordinate
  // @param coeff_d: polynomial coefficients for d coordinate
  // @param duration: max time duration. The max number of points is duration/time step.
  //
  void extendPath(std::vector<double> coeff_s, std::vector<double> coeff_d, double duration);

  // Print out the vehicle information around the ego car
  void printTraffic();

  std::vector<double> const* getPathS() const;

  std::vector<double> const* getPathD() const;

  double getMaxSpeed() const;

  double getMaxAcceleration() const;

  double getMaxJerk() const;

  double getMaxSteering() const;

  double getMinSafeDistance() const;

  Surroundings const* getSurroundings() const;

  Map const* getMap() const;

  int getTargetLaneID() const;

  void setTargetLaneID(int value);

  double getTargetSpeed() const;

  void setTargetSpeed(double value);

  long getTicker() const;
};


#endif //PATH_PLANNING_EGO_H
