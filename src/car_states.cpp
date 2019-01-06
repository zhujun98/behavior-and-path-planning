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
    case CLR:
      return new CarStateChangeLaneRight;
    case CLL:
      return new CarStateChangeLaneLeft;
    case FT:
      return new CarStateFollowTraffic;
    case ON:
      return new CarStateOn;
    default:
      throw std::invalid_argument("Unknown state!");
  }
}

/*
 * CarStateFollowTraffic class
 */

CarStateFollowTraffic::CarStateFollowTraffic() {
  transition_states_.push_back(CarTransitionStateFactory::createState(FT_TO_CLR));
  transition_states_.push_back(CarTransitionStateFactory::createState(FT_TO_CLL));
}

CarStateFollowTraffic::~CarStateFollowTraffic() = default;

void CarStateFollowTraffic::onEnter(Car& car) {
  std::cout << "Enter state: *** FOLLOW TRAFFIC ***" << std::endl;
}

void CarStateFollowTraffic::onUpdate(Car &car) {
  car.followTraffic();
}

void CarStateFollowTraffic::onExit(Car& car) {
  std::cout << "Exit state: *** FOLLOW TRAFFIC ***" << std::endl;
}

/*
 * CarStateReady class
 */

CarStateOn::CarStateOn() {
  transition_states_.push_back(CarTransitionStateFactory::createState(ON_TO_FT));
}

CarStateOn::~CarStateOn() = default;

void CarStateOn::onEnter(Car& car) {
  std::cout << "Enter state: *** ON ***" << std::endl;
}

void CarStateOn::onUpdate(Car &car) {
  car.followTraffic();
}

void CarStateOn::onExit(Car& car) {
  std::cout << "Exit state: *** ON ***" << std::endl;
}

/*
 * CarStateChangeLaneLeft class
 */

CarStateChangeLaneLeft::CarStateChangeLaneLeft() {
  transition_states_.push_back(CarTransitionStateFactory::createState(CL_TO_FT));
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
  transition_states_.push_back(CarTransitionStateFactory::createState(CL_TO_FT));
}

CarStateChangeLaneRight::~CarStateChangeLaneRight() = default;

void CarStateChangeLaneRight::onEnter(Car& car) {
  std::cout << "Enter state: *** CHANGE TO THE RIGHT LANE *** from Lane-" << car.getCurrentLaneId()
            << " to Lane-" << car.getTargetLaneId() << std::endl;
}

void CarStateChangeLaneLeft::onUpdate(Car& car) {}

void CarStateChangeLaneRight::onExit(Car& carp) {
  std::cout << "Exit state: *** CHANGE TO THE RIGHT LANE *** " << std::endl;
}