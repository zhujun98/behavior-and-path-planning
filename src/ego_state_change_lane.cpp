//
// Created by jun on 7/24/17.
//
#include <iostream>

#include "vehicle.h"
#include "ego.h"
#include "ego_state_change_lane.h"
#include "ego_state_keep_lane.h"
#include "map.h"


EgoStateChangeLane::EgoStateChangeLane(int target_lane_id) {
  target_lane_id_ = target_lane_id;
}

EgoStateChangeLane::~EgoStateChangeLane() {}

void EgoStateChangeLane::onEnter(Ego& ego) {
  std::cout << "Enter state: *** CHANGE LANE *** from Lane-" << ego.getLaneID()
            << " to Lane-" << target_lane_id_ << std::endl;
}

EgoState* EgoStateChangeLane::onUpdate(Ego& ego, const Map& map) {
  if ( ego.getLaneID() == target_lane_id_ ) {
    return new EgoStateKeepLane();
  } else {
    return nullptr;
  }
}

void EgoStateChangeLane::onExit(Ego& ego) {
  std::cout << "Exit state: *** CHANGE LANE *** " << std::endl;
}

void EgoStateChangeLane::planPath(Ego &ego, const Map& map) {
  ;
}