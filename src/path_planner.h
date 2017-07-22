//
// Created by jun on 7/21/17.
//

#ifndef PATH_PLANNING_PATH_PLANNER_H
#define PATH_PLANNING_PATH_PLANNER_H


#include <vector>

#include "vehicle.h"


class PathPlanner {

private:
  Ego *car_;

  int n_path_points_;  // No. of path points to predict
  double time_step_;  // time step between points

public:

  //
  // Constructor
  //
  PathPlanner(Ego& car);

  //
  // Destructor
  //
  ~PathPlanner();

  std::vector<double>
  jerk_minimizing_trajectory(std::vector<double> state0,
                             std::vector<double> state1, double dt) const;

  double eval_trajectory(std::vector<double> coeff, double t) const;

  //
  //
  //
  void plan();

  //
  //
  //
  void keep_lane();
};


#endif //PATH_PLANNING_PATH_PLANNER_H
