//
// Created by jun on 7/24/17.
//
#include <iostream>

#include "../vehicle.h"
#include "../ego.h"
#include "ego_state_change_lane.h"
#include "../map.h"
#include "../utilities.h"
#include "../ego_transition_states/ego_transition_state.h"
#include "../path_planner.h"


EgoStateChangeLane::EgoStateChangeLane() {}

EgoStateChangeLane::~EgoStateChangeLane() {}

void EgoStateChangeLane::onUpdate(Ego &ego) {
  // TODO: add collision check here.
}
