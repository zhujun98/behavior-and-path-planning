//
// Created by jun on 7/28/17.
//
#ifndef PATH_PLANNING_EGO_TRANSITION_FT_TO_CS_H
#define PATH_PLANNING_EGO_TRANSITION_FT_TO_CS_H

#include "ego_transition_state.h"


class Ego;
class EgoState;


class EgoTransitionFTToCS : public EgoTransitionState {
public:

  // constructor
  EgoTransitionFTToCS();

  // destructor
  ~EgoTransitionFTToCS();

  bool isValid(Ego& ego) const;

  EgoState* getNextState(Ego& ego) const;
};


#endif //PATH_PLANNING_EGO_TRANSITION_FT_TO_CS_H
