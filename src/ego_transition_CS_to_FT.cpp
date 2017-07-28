//
// Created by jun on 7/28/17.
//

#include "ego_transition_CS_to_FT.h"
#include "ego.h"
#include "ego_state.h"


EgoTransitionCSToFT::EgoTransitionCSToFT() {}

EgoTransitionCSToFT::~EgoTransitionCSToFT() {}

EgoState* EgoTransitionCSToFT::getNextState(Ego& ego) const {
  return EgoStateFactory::createState(FT);
}

bool EgoTransitionCSToFT::isValid(Ego &ego) const {
  return ( ego.getSpeed() < 0.95*ego.getTargetSpeed() );
}
