//
// Created by jun on 7/28/17.
//
#ifndef PATH_PLANNING_EGO_TRANSITION_FT_TO_CL_H
#define PATH_PLANNING_EGO_TRANSITION_FT_TO_CL_H

#include <vector>

#include "ego_transition_state.h"

class Ego;
class EgoState;


class EgoTransitionFTToCL : public EgoTransitionState {

protected:
  // constructor
  EgoTransitionFTToCL();

  //
  // check the whether the direction of lane change is optimal
  //
  // @param direction: +1 for to the right and -1 for to the left
  //
  // TODO:: there is plenty room to improve the AI
  bool isOptimal(const Ego& ego, int direction) const;

  //
  // Plan the path for the lane change. If a feasible path is found,
  // return true and set the path for the ego car. If not, return
  // false.
  //
  // @param direction: +1 for to the right and -1 for to the left
  //
  bool planPath(Ego &ego, int direction) const;

public:

  // destructor
  virtual ~EgoTransitionFTToCL();

};

#endif //PATH_PLANNING_EGO_TRANSITION_FT_TO_CL_H
