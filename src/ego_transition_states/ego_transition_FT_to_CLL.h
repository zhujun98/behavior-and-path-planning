//
// Created by jun on 7/29/17.
//
#ifndef PATH_PLANNING_EGO_TRANSITION_FT_TO_CLL_H
#define PATH_PLANNING_EGO_TRANSITION_FT_TO_CLL_H

#include "ego_transition_FT_to_CL.h"


class EgoTransitionFTToCLL : public EgoTransitionFTToCL {
public:

  // constructor
  EgoTransitionFTToCLL();

  // destructor
  ~EgoTransitionFTToCLL() override;

  bool isValid(Ego& ego) const override;

  EgoState* getNextState(Ego& ego) const override;
};


#endif //PATH_PLANNING_EGO_TRANSITION_FT_TO_CLL_H