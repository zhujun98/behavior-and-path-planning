//
// Created by jun on 7/29/17.
//
#include <iostream>

#include "ego_state_change_lane_right.h"
#include "../ego.h"
#include "../ego_transition_states/ego_transition_state.h"


EgoStateChangeLaneRight::EgoStateChangeLaneRight() {
  transition_states_.push_back(EgoTransitionStateFactory::createState(CL_TO_FT));
}

EgoStateChangeLaneRight::~EgoStateChangeLaneRight() {}

void EgoStateChangeLaneRight::onEnter(Ego& ego) {
  ego.truncatePath(15);
  planPath(ego);
  std::cout << "Enter state: *** CHANGE TO THE RIGHT LANE *** from Lane-" << ego.getLaneID()
            << " to Lane-" << ego.getTargetLaneID() << std::endl;
}

void EgoStateChangeLaneRight::onUpdate(Ego &ego) {
  ;
}

void EgoStateChangeLaneRight::onExit(Ego& egop) {
  std::cout << "Exit state: *** CHANGE TO THE RIGHT LANE *** " << std::endl;
}
