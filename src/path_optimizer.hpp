#ifndef BEHAVIOR_AND_PATH_PLANNING_PATH_OPTIMIZER_H
#define BEHAVIOR_AND_PATH_PLANNING_PATH_OPTIMIZER_H

#include <iostream>
#include <memory>

#include "common.hpp"
#include "map.hpp"


class PathOptimizer {

  double speed_limit_; // maximum speed (m/s)
  double acc_limit_; // maximum acceleration (m/s^2)
  double jerk_limit_; // maximum jerk (m/s^3)

  double time_step_;
  double time_limit_ = 4.0; // maximum time span of a path (s)

  double dist_step_ = 1.0; // distance step during search (m)
  // maximum distance is bounded by speed_limit_ * time_limit_

  std::shared_ptr<Map> map_;

  // other vehicles on the road ((s, vs, as), (d, vd, ad))
  std::shared_ptr<cars_on_road> vehicles_;

public:

  PathOptimizer(std::shared_ptr<Map> map,
                double time_step, double speed_limit, double acc_limit, double jerk_limit);

  ~PathOptimizer();

  /**
   * Validate a JMT path.
   */
  bool validatePath(const dynamics& dyn, const polynomial_coeff& coeff_s, const polynomial_coeff& coeff_d,
                    double delta_t);

  /**
   * Compute the trajectory in the Frenet coordinate system.
   */
  trajectory
  computeJmtPath(const polynomial_coeff& coeff_s, const polynomial_coeff& coeff_d, double delta_t);

  /**
   * Get the optimized path when starting up.
   */
  trajectory startUp(dynamics&& dyn);

  /**
   * Get the optimized path when keeping lane.
   */
  trajectory keepLane(dynamics&& dyn, const dynamics& front_vehicle);

  /**
   * Get the optimized path when changing lane.
   *
   * :param target_lane_id: target lane ID.
   */
  trajectory changeLane(dynamics&& dyn, const dynamics& front_vehicle, uint16_t target_lane_id);

  /**
   * Search the optimized JMT path for a given condition.
   *
   * :param dyn: initial vehicle dynamics
   * :param dyn_f: final vehicle dynamics with minimum allowed s
   */
  trajectory searchOptimizedJMT(dynamics&& dyn, dynamics&& dyn_f);
};


#endif //BEHAVIOR_AND_PATH_PLANNING_PATH_OPTIMIZER_H
