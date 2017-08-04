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
  // @param direction: 1 for to the right, otherwise to the left
  //
  bool willCollision(const Ego& ego, int direction) const;

  bool isOptimal(const Ego& ego, int direction) const;

  void planPath(Ego &ego) const;

public:

  // destructor
  virtual ~EgoTransitionFTToCL();

};

#endif //PATH_PLANNING_EGO_TRANSITION_FT_TO_CL_H
