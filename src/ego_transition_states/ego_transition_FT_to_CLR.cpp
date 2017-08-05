//
// Created by jun on 7/29/17.
//
#include "ego_transition_FT_to_CLR.h"
#include "../ego.h"
#include "../ego_states/ego_state.h"


EgoTransitionFTToCLR::EgoTransitionFTToCLR() {}

EgoTransitionFTToCLR::~EgoTransitionFTToCLR() {}

EgoState* EgoTransitionFTToCLR::getNextState(Ego& ego) const {
  return EgoStateFactory::createState(CLR);
}

bool EgoTransitionFTToCLR::isValid(Ego &ego) const {
  return ( isOptimal(ego, 1) && planPath(ego, 1) );
}
