//
// Created by jun on 7/21/17.
//

#ifndef PATH_PLANNING_PATH_PLANNER_H
#define PATH_PLANNING_PATH_PLANNER_H


#include <vector>

class PathPlanner {
private:

public:

  //
  // Constructor
  //
  PathPlanner();

  //
  // Destructor
  //
  ~PathPlanner();

  std::vector<double>
  jerk_minimizing_trajectory(std::vector<double> state0,
                             std::vector<double> state1, double dt) const;

  double eval_trajectory(std::vector<double> coeff, double t) const;
};


#endif //PATH_PLANNING_PATH_PLANNER_H
