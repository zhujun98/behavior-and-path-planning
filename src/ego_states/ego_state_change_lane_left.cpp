//
// Created by jun on 7/29/17.
//
#include <iostream>

#include "ego_state_change_lane_left.h"
#include "../ego.h"
#include "../ego_transition_states/ego_transition_state.h"


EgoStateChangeLaneLeft::EgoStateChangeLaneLeft() {
  transition_states_.push_back(EgoTransitionStateFactory::createState(CL_TO_FT));
}

EgoStateChangeLaneLeft::~EgoStateChangeLaneLeft() {}

void EgoStateChangeLaneLeft::onEnter(Ego& ego) {
  std::cout << "Enter state: *** CHANGE TO THE LEFT LANE *** from Lane-" << ego.getLaneID()
            << " to Lane-" << ego.getTargetLaneID() << std::endl;
}

void EgoStateChangeLaneLeft::onExit(Ego& egop) {
  std::cout << "Exit state: *** CHANGE TO THE LEFT LANE *** " << std::endl;
}
