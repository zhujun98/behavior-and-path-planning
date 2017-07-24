//
// Created by jun on 7/21/17.
//

#ifndef PATH_PLANNING_PATH_PLANNER_H
#define PATH_PLANNING_PATH_PLANNER_H

#include <vector>

#include "map.h"


class Ego;


class PathPlanner {

private:
  Ego* ego_;
  const Map* map_;

  int max_prediction_points_;  // No. of path points to predict
  double time_step_;  // time step between points

public:

  //
  // Constructor
  //
  PathPlanner(Ego& car, const Map& map);

  //
  // Destructor
  //
  ~PathPlanner();

  //
  // Find the coefficients of a quinted polynomial which minimizes the
  // jerk between the initial state and the final state in a given period.
  // Note:: for multi-dimensional scenario, one needs to apply this
  //        function to different directions separately.
  //
  // @param state0: initial state [x, dx/dt, d^2(x)/dt^2]
  // @param state1: final state [x, dx/dt, d^2(x)/dt^2]
  // @param dt: transition time (s) between the initial and final states
  //
  std::vector<double>
  jerkMinimizingTrajectory(std::vector<double> state0,
                             std::vector<double> state1, double dt) const;

  //
  // Evaluate the position at time t using the given polynomial coefficients
  // y = p[0] + p[1]*t + p[2]*t + ... + p[n-1]*t^(n-1)
  //
  // @param p: polynomial coefficients
  // @param t: time
  //
  // @return: position at time t
  //
  double evalTrajectory(std::vector<double> p, double t) const;

  //
  // Plan the path according to the behavior state
  //
  void plan();

  //
  // Extend path (s, d) to the existing path (s, d) by duration
  // t using the given polynomial coefficients.
  //
  // @param coeff_s: polynomial coefficients for the s coordinate
  // @param coeff_d: polynomial coefficients for the d coordinate
  //
  void extendPath(std::vector<double> coeff_s, std::vector<double> coeff_d);

  //
  //
  //
  void keepLane();

  //
  //
  //
  void prepareLaneChange();
};


#endif //PATH_PLANNING_PATH_PLANNER_H
