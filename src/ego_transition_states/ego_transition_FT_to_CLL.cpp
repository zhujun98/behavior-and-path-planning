//
// Created by jun on 7/29/17.
//
#include "ego_transition_FT_to_CLL.h"
#include "../ego.h"
#include "../ego_states/ego_state.h"


EgoTransitionFTToCLL::EgoTransitionFTToCLL() {}

EgoTransitionFTToCLL::~EgoTransitionFTToCLL() {}

EgoState* EgoTransitionFTToCLL::getNextState(Ego& ego) const {
  return EgoStateFactory::createState(CLL);
}

bool EgoTransitionFTToCLL::isValid(Ego &ego) const {
  return ( isOptimal(ego, -1) && planPath(ego, -1) );
}