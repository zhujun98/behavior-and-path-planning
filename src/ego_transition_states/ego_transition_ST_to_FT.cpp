//
// Created by jun on 7/31/17.
//
#include "../ego.h"
#include "../ego_states/ego_state.h"
#include "ego_transition_ST_to_FT.h"


EgoTransitionSTToFT::EgoTransitionSTToFT() {}

EgoTransitionSTToFT::~EgoTransitionSTToFT() {}

EgoState* EgoTransitionSTToFT::getNextState(Ego& ego) const {
  return EgoStateFactory::createState(FT);
}

bool EgoTransitionSTToFT::isValid(Ego &ego) const {
  // return true if speed > 5*2.25 = 11.25 MPH
  return ( ego.getSpeed() > 5 );
}
