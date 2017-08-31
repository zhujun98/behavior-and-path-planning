//
// Created by jun on 7/28/17.
//
#ifndef PATH_PLANNING_EGO_TRANSITION_CL_TO_FT_H
#define PATH_PLANNING_EGO_TRANSITION_CL_TO_FT_H

#include "ego_transition_state.h"

class Ego;
class EgoState;


class EgoTransitionCLToFT : public EgoTransitionState {
public:

  // constructor
  EgoTransitionCLToFT();

  // destructor
  ~EgoTransitionCLToFT() override;

  bool isValid(Ego& ego) const override;

  EgoState* getNextState(Ego& ego) const override;
};


#endif //PATH_PLANNING_EGO_TRANSITION_CL_TO_FT_H