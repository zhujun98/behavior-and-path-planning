//
// Created by jun on 7/24/17.
//
#include <iostream>

#include "ego.h"
#include "ego_state.h"
#include "ego_transition_state.h"
#include "ego_state_change_lane.h"
#include "ego_state_constant_speed.h"
#include "ego_state_follow_traffic.h"


/*
 * EgoState class
 */
EgoState::EgoState() {}

EgoState::~EgoState() {}

EgoState* EgoState::checkTransition(Ego &ego) {
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
    case CL:
      return new EgoStateChangeLane;
    case CS:
      return new EgoStateConstantSpeed;
    case FT:
      return new EgoStateFollowTraffic;
    default:
      throw std::invalid_argument("Unknown state!");
  }
}
