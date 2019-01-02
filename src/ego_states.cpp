//
// Created by jun on 7/24/17.
//
#include <iostream>

#include "ego.hpp"
#include "map.hpp"
#include "ego_states.hpp"
#include "ego_transition_states.hpp"


/*
 * EgoState class
 */

EgoState::EgoState() : timer_(0.0) {}

EgoState::~EgoState() = default;

EgoState* EgoState::checkTransition(Ego &ego) {
  ++timer_;
  // avoid frequently switching between states
  if (timer_ < 5) return nullptr;

  for (const auto& v : transition_states_)
    if ( v->isValid(ego) ) return v->getNextState(ego);

  return nullptr;
}


/*
 * EgoStateFactory class
 */

EgoStateFactory::EgoStateFactory() = default;

EgoStateFactory::~EgoStateFactory() = default;

EgoState* EgoStateFactory::createState(States name) {
  switch(name) {
    case CLR:
      return new EgoStateChangeLaneRight;
    case CLL:
      return new EgoStateChangeLaneLeft;
    case FT:
      return new EgoStateFollowTraffic;
    default:
      throw std::invalid_argument("Unknown state!");
  }
}


/*
 * EgoStateFollowTraffic class
 */

EgoStateFollowTraffic::EgoStateFollowTraffic() {
  transition_states_.push_back(EgoTransitionStateFactory::createState(FT_TO_CLR));
  transition_states_.push_back(EgoTransitionStateFactory::createState(FT_TO_CLL));
}

EgoStateFollowTraffic::~EgoStateFollowTraffic() = default;

void EgoStateFollowTraffic::onEnter(Ego& ego) {
  std::cout << "Enter state: *** FOLLOW TRAFFIC ***" << std::endl;
}

void EgoStateFollowTraffic::onUpdate(Ego &ego) {
  ego.followTraffic();
}

void EgoStateFollowTraffic::onExit(Ego& ego) {
  std::cout << "Exit state: *** FOLLOW TRAFFIC ***" << std::endl;
}

/*
 * EgoStateChangeLaneLeft class
 */

EgoStateChangeLaneLeft::EgoStateChangeLaneLeft() {
  transition_states_.push_back(EgoTransitionStateFactory::createState(CL_TO_FT));
}

EgoStateChangeLaneLeft::~EgoStateChangeLaneLeft() = default;

void EgoStateChangeLaneLeft::onEnter(Ego& ego) {
  std::cout << "Enter state: *** CHANGE TO THE LEFT LANE *** from Lane-" << ego.getCurrentLaneId()
            << " to Lane-" << ego.getTargetLaneId() << std::endl;
}

void EgoStateChangeLaneRight::onUpdate(Ego& ego) {}

void EgoStateChangeLaneLeft::onExit(Ego& egop) {
  std::cout << "Exit state: *** CHANGE TO THE LEFT LANE *** " << std::endl;
}

/*
 * EgoStateChangeLaneRight class
 */

EgoStateChangeLaneRight::EgoStateChangeLaneRight() {
  transition_states_.push_back(EgoTransitionStateFactory::createState(CL_TO_FT));
}

EgoStateChangeLaneRight::~EgoStateChangeLaneRight() = default;

void EgoStateChangeLaneRight::onEnter(Ego& ego) {
  std::cout << "Enter state: *** CHANGE TO THE RIGHT LANE *** from Lane-" << ego.getCurrentLaneId()
            << " to Lane-" << ego.getTargetLaneId() << std::endl;
}

void EgoStateChangeLaneLeft::onUpdate(Ego& ego) {}

void EgoStateChangeLaneRight::onExit(Ego& egop) {
  std::cout << "Exit state: *** CHANGE TO THE RIGHT LANE *** " << std::endl;
}