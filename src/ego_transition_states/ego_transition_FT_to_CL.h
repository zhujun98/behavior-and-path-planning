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

  bool isOptimal(const Ego& ego, int direction) const;

  bool planPath(Ego &ego, int direction) const;

public:

  // destructor
  virtual ~EgoTransitionFTToCL();

};

#endif //PATH_PLANNING_EGO_TRANSITION_FT_TO_CL_H
