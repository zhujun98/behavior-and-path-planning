//
// Created by jun on 7/28/17.
//
#include <iostream>

#include "ego_transition_state.h"
#include "ego_transition_CS_to_FT.h"
#include "ego_transition_CL_to_FT.h"
#include "ego_transition_FT_to_CS.h"
#include "ego_transition_FT_to_CL.h"

/*
 * EgoTransitionState class
 */
EgoTransitionState::EgoTransitionState() {}

EgoTransitionState::~EgoTransitionState() {}


/*
 * EgoTransitionStateFactory class
 */

EgoTransitionStateFactory::EgoTransitionStateFactory() {}

EgoTransitionStateFactory::~EgoTransitionStateFactory() {}

EgoTransitionState* EgoTransitionStateFactory::createState(TransitionStates name) {
  switch (name) {
    case CS_TO_FT:
      return new EgoTransitionCSToFT;
    case CL_TO_FT:
      return new EgoTransitionCLToFT;
    case FT_TO_CL:
      return new EgoTransitionFTToCL;
    case FT_TO_CS:
      return new EgoTransitionFTToCS;
    default:
      throw std::invalid_argument("Unknown transition state!");
  }
}