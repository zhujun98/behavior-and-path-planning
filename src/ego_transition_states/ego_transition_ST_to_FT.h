//
// Created by jun on 7/31/17.
//
#ifndef PATH_PLANNING_EGO_TRANSITION_ST_TO_FT_H
#define PATH_PLANNING_EGO_TRANSITION_ST_TO_FT_H

#include "ego_transition_state.h"


class Ego;
class EgoState;


class EgoTransitionSTToFT : public EgoTransitionState {

public:
  // constructor
  EgoTransitionSTToFT();

  // destructor
  ~EgoTransitionSTToFT() override;

  bool isValid(Ego& ego) const override;

  EgoState* getNextState(Ego& ego) const override;

};


#endif //PATH_PLANNING_EGO_TRANSITION_ST_TO_FT_H