//
// Created by jun on 7/22/17.
//

#ifndef PATH_PLANNING_BEHAVIOR_PLANNER_H
#define PATH_PLANNING_BEHAVIOR_PLANNER_H

#include "vehicle.h"


class BehaviorPlanner {

private:

  Ego* ego_;

public:

  //
  // constructor
  //
  BehaviorPlanner(Ego& ego);

  //
  // destructor
  //
  ~BehaviorPlanner();

  void plan();
};


#endif //PATH_PLANNING_BEHAVIOR_PLANNER_H
