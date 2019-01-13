//
// Created by jun on 7/24/17.
//
#include <iostream>

#include "car.hpp"
#include "map.hpp"
#include "car_states.hpp"
#include "car_transition_states.hpp"


/*
 * CarState class
 */

CarState::CarState() : timer_(0.0) {}

CarState::~CarState() = default;

CarState* CarState::checkTransition(Car &car) {
  ++timer_;
  // avoid frequently switching between states
  if (timer_ < 5) return nullptr;

  for (const auto& v : transition_states_)
    if ( v->isValid(car) ) return v->getNextState(car);

  return nullptr;
}


/*
 * CarStateFactory class
 */

CarStateFactory::CarStateFactory() = default;

CarStateFactory::~CarStateFactory() = default;

CarState* CarStateFactory::createState(States name) {
  switch(name) {
    case States::CLR:
      return new CarStateChangeLaneRight;
    case States::CLL:
      return new CarStateChangeLaneLeft;
    case States::KL:
      return new CarStateKeepLane;
    case States::ON:
      return new CarStateOn;
    default:
      throw std::invalid_argument("Unknown state!");
  }
}

/*
 * CarStateFollowTraffic class
 */

CarStateKeepLane::CarStateKeepLane() {
  transition_states_.push_back(CarTransitionStateFactory::createState(TransitionStates::KL_TO_CLR));
  transition_states_.push_back(CarTransitionStateFactory::createState(TransitionStates::KL_TO_CLL));
}

CarStateKeepLane::~CarStateKeepLane() = default;

void CarStateKeepLane::onEnter(Car& car) {
  std::cout << "Enter state: *** KEEP LANE ***" << std::endl;
}

void CarStateKeepLane::onUpdate(Car &car) {
  car.planPath();
}

void CarStateKeepLane::onExit(Car& car) {
  std::cout << "Exit state: *** KEEP LANE ***" << std::endl;
}

/*
 * CarStateReady class
 */

CarStateOn::CarStateOn() {
  transition_states_.push_back(CarTransitionStateFactory::createState(TransitionStates::ON_TO_KL));
}

CarStateOn::~CarStateOn() = default;

void CarStateOn::onEnter(Car& car) {
  std::cout << "Enter state: *** ON ***" << std::endl;
}

void CarStateOn::onUpdate(Car &car) {
  car.planPath();
}

void CarStateOn::onExit(Car& car) {
  std::cout << "Exit state: *** ON ***" << std::endl;
}

/*
 * CarStateChangeLaneLeft class
 */

CarStateChangeLaneLeft::CarStateChangeLaneLeft() {
  transition_states_.push_back(CarTransitionStateFactory::createState(TransitionStates::CL_TO_KL));
}

CarStateChangeLaneLeft::~CarStateChangeLaneLeft() = default;

void CarStateChangeLaneLeft::onEnter(Car& car) {
  std::cout << "Enter state: *** CHANGE TO THE LEFT LANE *** from Lane-" << car.getCurrentLaneId()
            << " to Lane-" << car.getTargetLaneId() << std::endl;
}

void CarStateChangeLaneRight::onUpdate(Car& car) {}

void CarStateChangeLaneLeft::onExit(Car& car) {
  std::cout << "Exit state: *** CHANGE TO THE LEFT LANE *** " << std::endl;
}

/*
 * CarStateChangeLaneRight class
 */

CarStateChangeLaneRight::CarStateChangeLaneRight() {
  transition_states_.push_back(CarTransitionStateFactory::createState(TransitionStates::CL_TO_KL));
}

CarStateChangeLaneRight::~CarStateChangeLaneRight() = default;

void CarStateChangeLaneRight::onEnter(Car& car) {
  std::cout << "Enter state: *** CHANGE TO THE RIGHT LANE *** from Lane-" << car.getCurrentLaneId()
            << " to Lane-" << car.getTargetLaneId() << std::endl;
}

void CarStateChangeLaneLeft::onUpdate(Car& car) {
  car.planPath();
}

void CarStateChangeLaneRight::onExit(Car& carp) {
  std::cout << "Exit state: *** CHANGE TO THE RIGHT LANE *** " << std::endl;
}