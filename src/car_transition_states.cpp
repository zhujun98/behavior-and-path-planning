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
    case TransitionStates::ON_TO_KL:
      static CarTransitionON2KL ready_to_kl;
      return &ready_to_kl;
    case TransitionStates::CL_TO_KL:
      static CarTransitionCL2KL cl_to_kl;
      return &cl_to_kl;
    case TransitionStates::KL_TO_CLL:
      static CarTransitionKL2CLR kl_to_cll;
      return &kl_to_cll;
    case TransitionStates::KL_TO_CLR:
      static CarTransitionKL2CLR kl_to_clr;
      return &kl_to_clr;
    default:
      throw std::invalid_argument("Unknown transition state!");
  }
}

/* CarTransitionON2KL */

CarTransitionON2KL::CarTransitionON2KL() = default;

CarTransitionON2KL::~CarTransitionON2KL() = default;

CarState* CarTransitionON2KL::getNextState(Car& car) const {
  return CarStateFactory::createState(States::KL);
}

bool CarTransitionON2KL::isValid(Car &car) const {
  return true;
}

/* CarTransitionCL2KL */

CarTransitionCL2KL::CarTransitionCL2KL() = default;

CarTransitionCL2KL::~CarTransitionCL2KL() = default;

CarState* CarTransitionCL2KL::getNextState(Car& car) const {
  return CarStateFactory::createState(States::KL);
}

bool CarTransitionCL2KL::isValid(Car &car) const {
  return car.getTargetLaneId() == car.getCurrentLaneId();
}

/* CarTransitionKL2CLL */

CarTransitionKL2CLL::CarTransitionKL2CLL() = default;

CarTransitionKL2CLL::~CarTransitionKL2CLL() = default;

CarState* CarTransitionKL2CLL::getNextState(Car& car) const {
  return CarStateFactory::createState(States::CLL);
}

bool CarTransitionKL2CLL::isValid(Car &car) const {
  return false;
}

/* CarTransitionKL2CLR */

CarTransitionKL2CLR::CarTransitionKL2CLR() = default;

CarTransitionKL2CLR::~CarTransitionKL2CLR() = default;

CarState* CarTransitionKL2CLR::getNextState(Car& car) const {
  return CarStateFactory::createState(States::CLR);
}

bool CarTransitionKL2CLR::isValid(Car &car) const {
  return false;
}
