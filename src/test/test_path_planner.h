//
// Created by jun on 7/29/17.
//

#ifndef PATH_PLANNING_TEST_PATH_PLANNER_H
#define PATH_PLANNING_TEST_PATH_PLANNER_H

#include "../path_planner.h"
#include "../utilities.h"

inline void testPathPlanner() {

  std::cout << "Testing PathPlanner class" << std::endl;

  PathPlanner planner(22, 10, 10);

  vehicle_state state0 {{0,  0, 0}, {1, 0, 0}};
  vehicle_state state1 {{10, 0, 0}, {0, 0, 0}};
  double duration = 10;

  vehicle_trajectory new_path = planner.plan(state0, duration);

  print1DContainer(new_path.first);
  print1DContainer(new_path.second);

  std::cout << "passed!" << std::endl;

}


#endif //PATH_PLANNING_TEST_PATH_PLANNER_H
