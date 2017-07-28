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


struct Surroundings {
  std::vector<std::vector<double>> left;
  std::vector<std::vector<double>> right;
  std::vector<std::vector<double>> center;
};


class Ego : public Vehicle {

private:
  EgoState* state_;

  Surroundings surroundings_;

  int target_lane_id_;
  double target_speed_;

  std::vector<double> path_s_;
  std::vector<double> path_d_;

  double min_safe_distance_;

  double max_speed_;  // maximum speed (m/s)
  double max_acceleration_;  // maximum acceleration (m/s^2)
  double max_steering_;  // maximum steering angle (rad)

public:

  //
  // constructor
  //
  Ego(const Map& map);

  //
  // destructor
  //
  virtual ~Ego();

  //
  // Update the ego car's state
  //
  void update(const std::vector<double>& localization,
              const std::vector<std::vector<double>>& sensor_fusion);

  //
  // Remove the way points in the vehicle paths which have been processed.
  //
  void updateUnprocessedPath();

  //
  //
  //
  void updateSurroundings(const std::vector<std::vector<double>>& sensor_fusion);

  //
  // Keep a given number of way points in the current path
  //
  void truncatePath(unsigned int n_keep);

  //
  //
  //
  void extendPath(std::vector<double> coeff_s, std::vector<double> coeff_d, double duration);

  std::vector<double> const* getPathS() const;

  std::vector<double> const* getPathD() const;

  double getMaxSpeed() const;

  double getMaxAcceleration() const;

  double getMaxSteering() const;

  double getMinSafeDistance() const;

  Surroundings const* getSurroundings() const;

  Map const* getMap() const;

  int getTargetLaneID() const;

  void setTargetLaneID(int value);

  double getTargetSpeed() const;

  void setTargetSpeed(double value);
};


#endif //PATH_PLANNING_EGO_H
