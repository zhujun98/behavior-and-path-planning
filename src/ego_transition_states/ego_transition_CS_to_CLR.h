//
// Created by jun on 7/29/17.
//
#ifndef PATH_PLANNING_EGO_TRANSITION_CS_TO_CLR_H
#define PATH_PLANNING_EGO_TRANSITION_CS_TO_CLR_H

#include "ego_transition_FT_to_CLR.h"


class EgoTransitionCSToCLR : public EgoTransitionFTToCLR {

public:
  // constructor
  EgoTransitionCSToCLR();

  // destructor
  ~EgoTransitionCSToCLR() override;

};


#endif //PATH_PLANNING_EGO_TRANSITION_CS_TO_CLR_H