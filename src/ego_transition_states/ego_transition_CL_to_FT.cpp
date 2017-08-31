//
// Created by jun on 7/28/17.
//

#include "ego_transition_CL_to_FT.h"
#include "../ego.h"
#include "../ego_states/ego_state.h"


EgoTransitionCLToFT::EgoTransitionCLToFT() {}

EgoTransitionCLToFT::~EgoTransitionCLToFT() {}

EgoState* EgoTransitionCLToFT::getNextState(Ego& ego) const {
  return EgoStateFactory::createState(FT);
}

bool EgoTransitionCLToFT::isValid(Ego &ego) const {
  return ( ego.getTargetLaneID() == ego.getLaneID() );
}