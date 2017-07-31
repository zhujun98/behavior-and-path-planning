//
// Created by jun on 7/31/17.
//

#ifndef PATH_PLANNING_EGO_TRANSITION_ST_TO_FT_H
#define PATH_PLANNING_EGO_TRANSITION_ST_TO_FT_H


#include <vector>

#include "ego_transition_state.h"


class Ego;
class EgoState;


class EgoTransitionSTToFT : public EgoTransitionState {

public:
  // constructor
  EgoTransitionSTToFT();

  // destructor
  ~EgoTransitionSTToFT();

  bool isValid(Ego& ego) const;

  EgoState* getNextState(Ego& ego) const;

};


#endif //PATH_PLANNING_EGO_TRANSITION_ST_TO_FT_H
