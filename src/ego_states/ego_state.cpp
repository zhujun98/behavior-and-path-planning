//
// Created by jun on 7/24/17.
//
#include <iostream>

#include "../ego.h"
#include "ego_state.h"
#include "../ego_transition_states/ego_transition_state.h"
#include "ego_state_start.h"
#include "ego_state_change_lane_right.h"
#include "ego_state_change_lane_left.h"
#include "ego_state_constant_speed.h"
#include "ego_state_follow_traffic.h"


/*
 * EgoState class
 */
EgoState::EgoState() {
  timer_ = 0;
}

EgoState::~EgoState() {}

EgoState* EgoState::checkTransition(Ego &ego) {
  ++ timer_;
  // avoid frequently switching between states
  if ( timer_ < 10 ) { return nullptr; }

  for ( const auto& v : transition_states_ ) {
    if ( v->isValid(ego) ) { return v->getNextState(ego); }
  }

  return nullptr;
}


/*
 * EgoStateFactory class
 */
EgoStateFactory::EgoStateFactory() {}

EgoStateFactory::~EgoStateFactory() {}

EgoState* EgoStateFactory::createState(States name) {
  switch(name) {
    case ST:
      return new EgoStateStart;
    case CLR:
      return new EgoStateChangeLaneRight;
    case CLL:
      return new EgoStateChangeLaneLeft;
    case CS:
      return new EgoStateConstantSpeed;
    case FT:
      return new EgoStateFollowTraffic;
    default:
      throw std::invalid_argument("Unknown state!");
  }
}
