//
// Created by jun on 7/29/17.
//
#ifndef PATH_PLANNING_EGO_TRANSITION_FT_TO_CLR_H
#define PATH_PLANNING_EGO_TRANSITION_FT_TO_CLR_H

#include "ego_transition_FT_to_CL.h"


class EgoTransitionFTToCLR : public EgoTransitionFTToCL {
public:

  // constructor
  EgoTransitionFTToCLR();

  // destructor
  virtual ~EgoTransitionFTToCLR();

  bool isValid(Ego& ego) const;

  EgoState* getNextState(Ego& ego) const;
};


#endif //PATH_PLANNING_EGO_TRANSITION_FT_TO_CLR_H
