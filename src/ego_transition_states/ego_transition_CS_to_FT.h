//
// Created by jun on 7/28/17.
//
#ifndef PATH_PLANNING_EGO_TRANSITION_CS_TO_CL_H
#define PATH_PLANNING_EGO_TRANSITION_CS_TO_CL_H

#include <vector>

#include "ego_transition_state.h"

class Ego;
class EgoState;


class EgoTransitionCSToFT : public EgoTransitionState {
public:
  // constructor
  EgoTransitionCSToFT();

  // destructor
  ~EgoTransitionCSToFT() override;

  bool isValid(Ego& ego) const override;

  EgoState* getNextState(Ego& ego) const override;
};


#endif //PATH_PLANNING_EGO_TRANSITION_CS_TO_CL_H