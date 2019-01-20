//
// Created by jun on 1/20/19.
//

#ifndef PROJECT_PATH_OPTIMIZER_H
#define PROJECT_PATH_OPTIMIZER_H

#include <iostream>
#include <vector>

#include "utilities.hpp"

/*
 * PathOptimizer
 */

class PathOptimizer {

  double speed_limit_; // maximum speed (m/s)
  double acc_limit_; // maximum acceleration (m/s^2)
  double jerk_limit_; // maximum jerk (m/s^3)

  double time_step_;

public:

  PathOptimizer(double speed_limit, double acc_limit, double jerk_limit, double time_step);

  ~PathOptimizer();

  /**
   * Validate a JMT path.
   */
  bool validatePath(const polynomial_coeff& coeff_s, const polynomial_coeff& coeff_d, double delta_t);

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
  trajectory keepLane(dynamics&& dyn, const dynamics& closest_front_car, double pd_f);

  /**
   * Get the optimized path when changing lane.
   */
  trajectory changeLane(dynamics&& dyn, const dynamics& closest_front_car, double pd_f);

  /**
   * Search the optimized JMT path for a given condition.
   *
   * :param dyn: initial vehicle dynamics
   * :param ps_f: final longitudinal position
   * :param pd_f: final transverse position
   * :param vs_f: final longitudinal velocity
   * :param vd_f: final transverse velocity
   * :param dist_step: step in searching valid ps (in meter).
   * :param time_limit: time span limit for the path
   */
  trajectory
  searchOptimizedJMT(dynamics&& dyn, double ps_f, double vs_f, double pd_f, double vd_f,
                     double dist_step, double time_limit);

};


#endif //PROJECT_PATH_OPTIMIZER_H
