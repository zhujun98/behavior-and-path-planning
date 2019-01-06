//
// Created by jun on 7/28/17.
//
#include <iostream>
#include <cmath>

#include "car.hpp"
#include "map.hpp"
#include "car_states.hpp"
#include "car_transition_states.hpp"


/* CarTransitionState */

CarTransitionState::CarTransitionState() = default;

CarTransitionState::~CarTransitionState() = default;

/* CarTransitionStateFactory */

CarTransitionStateFactory::CarTransitionStateFactory() = default;

CarTransitionStateFactory::~CarTransitionStateFactory() = default;

CarTransitionState* CarTransitionStateFactory::createState(TransitionStates name) {
  switch (name) {
    case CL_TO_FT:
      static CarTransitionCL2FT cl_to_ft;
      return &cl_to_ft;
    case FT_TO_CLL:
      static CarTransitionFT2CLR ft_to_cll;
      return &ft_to_cll;
    case FT_TO_CLR:
      static CarTransitionFT2CLR ft_to_clr;
      return &ft_to_clr;
    default:
      throw std::invalid_argument("Unknown transition state!");
  }
}

/* CarTransitionCL2FT */

CarTransitionCL2FT::CarTransitionCL2FT() = default;

CarTransitionCL2FT::~CarTransitionCL2FT() = default;

CarState* CarTransitionCL2FT::getNextState(Car& car) const {
  return CarStateFactory::createState(FT);
}

bool CarTransitionCL2FT::isValid(Car &car) const {
  return car.getTargetLaneId() == car.getCurrentLaneId();
}

/* CarTransitionFT2CLL */

CarTransitionFT2CLL::CarTransitionFT2CLL() = default;

CarTransitionFT2CLL::~CarTransitionFT2CLL() = default;

CarState* CarTransitionFT2CLL::getNextState(Car& car) const {
  return CarStateFactory::createState(CLL);
}

bool CarTransitionFT2CLL::isValid(Car &car) const {
}

/* CarTransitionFT2CLR */

CarTransitionFT2CLR::CarTransitionFT2CLR() = default;

CarTransitionFT2CLR::~CarTransitionFT2CLR() = default;

CarState* CarTransitionFT2CLR::getNextState(Car& car) const {
  return CarStateFactory::createState(CLR);
}

bool CarTransitionFT2CLR::isValid(Car &car) const {
}
