//
// Created by jun on 7/28/17.
//

#include "ego_transition_FT_to_CS.h"
#include "ego.h"
#include "ego_state.h"


EgoTransitionFTToCS::EgoTransitionFTToCS() {}

EgoTransitionFTToCS::~EgoTransitionFTToCS() {}

EgoState* EgoTransitionFTToCS::getNextState(Ego& ego) const {
  ego.setTargetSpeed(ego.getMaxSpeed());

  return EgoStateFactory::createState(CS);
}

bool EgoTransitionFTToCS::isValid(Ego &ego) const {
  return ( ego.getSpeed() >= ego.getMaxSpeed() );
}
