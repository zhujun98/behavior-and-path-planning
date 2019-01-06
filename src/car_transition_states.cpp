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
    case TransitionStates::ON_TO_FT:
      static CarTransitionON2FT ready_to_ft;
      return &ready_to_ft;
    case TransitionStates::CL_TO_FT:
      static CarTransitionCL2FT cl_to_ft;
      return &cl_to_ft;
    case TransitionStates::FT_TO_CLL:
      static CarTransitionFT2CLR ft_to_cll;
      return &ft_to_cll;
    case TransitionStates::FT_TO_CLR:
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
  return CarStateFactory::createState(States::FT);
}

bool CarTransitionCL2FT::isValid(Car &car) const {
  return car.getTargetLaneId() == car.getCurrentLaneId();
}


/* CarTransitionON2FT */

CarTransitionON2FT::CarTransitionON2FT() = default;

CarTransitionON2FT::~CarTransitionON2FT() = default;

CarState* CarTransitionON2FT::getNextState(Car& car) const {
  return CarStateFactory::createState(States::FT);
}

bool CarTransitionON2FT::isValid(Car &car) const {
  return true;
}


/* CarTransitionFT2CLL */

CarTransitionFT2CLL::CarTransitionFT2CLL() = default;

CarTransitionFT2CLL::~CarTransitionFT2CLL() = default;

CarState* CarTransitionFT2CLL::getNextState(Car& car) const {
  return CarStateFactory::createState(States::CLL);
}

bool CarTransitionFT2CLL::isValid(Car &car) const {
  return false;
}

/* CarTransitionFT2CLR */

CarTransitionFT2CLR::CarTransitionFT2CLR() = default;

CarTransitionFT2CLR::~CarTransitionFT2CLR() = default;

CarState* CarTransitionFT2CLR::getNextState(Car& car) const {
  return CarStateFactory::createState(States::CLR);
}

bool CarTransitionFT2CLR::isValid(Car &car) const {
  return false;
}
