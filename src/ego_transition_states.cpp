//
// Created by jun on 7/28/17.
//
#include <iostream>
#include <cmath>

#include "ego.hpp"
#include "map.hpp"
#include "ego_states.hpp"
#include "ego_transition_states.hpp"


/* EgoTransitionState */

EgoTransitionState::EgoTransitionState() = default;

EgoTransitionState::~EgoTransitionState() = default;

/* EgoTransitionStateFactory */

EgoTransitionStateFactory::EgoTransitionStateFactory() = default;

EgoTransitionStateFactory::~EgoTransitionStateFactory() = default;

EgoTransitionState* EgoTransitionStateFactory::createState(TransitionStates name) {
  switch (name) {
    case CL_TO_FT:
      static EgoTransitionCL2FT cl_to_ft;
      return &cl_to_ft;
    case FT_TO_CLL:
      static EgoTransitionFT2CLR ft_to_cll;
      return &ft_to_cll;
    case FT_TO_CLR:
      static EgoTransitionFT2CLR ft_to_clr;
      return &ft_to_clr;
    default:
      throw std::invalid_argument("Unknown transition state!");
  }
}

/* EgoTransitionCL2FT */

EgoTransitionCL2FT::EgoTransitionCL2FT() = default;

EgoTransitionCL2FT::~EgoTransitionCL2FT() = default;

EgoState* EgoTransitionCL2FT::getNextState(Ego& ego) const {
  return EgoStateFactory::createState(FT);
}

bool EgoTransitionCL2FT::isValid(Ego &ego) const {
  return ego.getTargetLaneId() == ego.getCurrentLaneId();
}

/* EgoTransitionFT2CLL */

EgoTransitionFT2CLL::EgoTransitionFT2CLL() = default;

EgoTransitionFT2CLL::~EgoTransitionFT2CLL() = default;

EgoState* EgoTransitionFT2CLL::getNextState(Ego& ego) const {
  return EgoStateFactory::createState(CLL);
}

bool EgoTransitionFT2CLL::isValid(Ego &ego) const {
}

/* EgoTransitionFT2CLR */

EgoTransitionFT2CLR::EgoTransitionFT2CLR() = default;

EgoTransitionFT2CLR::~EgoTransitionFT2CLR() = default;

EgoState* EgoTransitionFT2CLR::getNextState(Ego& ego) const {
  return EgoStateFactory::createState(CLR);
}

bool EgoTransitionFT2CLR::isValid(Ego &ego) const {
}
