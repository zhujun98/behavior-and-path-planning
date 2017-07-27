//
// Created by jun on 7/25/17.
//
#include <iostream>

#include "utilities.h"
#include "map.h"
#include "vehicle.h"
#include "ego.h"
#include "ego_state.h"
#include "ego_state_change_lane.h"
#include "ego_state_prepare_change_lane.h"
#include "ego_state_follow_traffic.h"


EgoStatePrepareChangeLane::EgoStatePrepareChangeLane() {}

EgoStatePrepareChangeLane::~EgoStatePrepareChangeLane() {}

void EgoStatePrepareChangeLane::onEnter(Ego& ego) {
  std::cout << "Enter state: *** PREPARE CHANGE LANE ***" << std::endl;
}

EgoState* EgoStatePrepareChangeLane::onUpdate(Ego& ego) {
  if ( checkCollision(ego) ) {
    return new EgoStateFollowTraffic();
  } else {
    return new EgoStateChangeLane(target_lane_id_);
  }
}

void EgoStatePrepareChangeLane::onExit(Ego& ego) {
  std::cout << "Exist state: *** PREPARE CHANGE LANE ***" << std::endl;
}

void EgoStatePrepareChangeLane::planPath(Ego& ego) {
  ;
}

bool EgoStatePrepareChangeLane::checkCollision(const Ego& ego) {
  // try left
  if (ego.getLaneID() > 1 && !checkSideCollision(ego, ego.getSurroundings()->left)) {
    target_lane_id_ = ego.getLaneID() - 1;
    return false;
  // try right
  } else if (ego.getLaneID() < 3 && !checkSideCollision(ego, ego.getSurroundings()->right)) {
    target_lane_id_ = ego.getLaneID() + 1;
    return false;
  }

  return true;
}

bool EgoStatePrepareChangeLane::
checkSideCollision(const Ego& ego, std::vector<std::vector<double>> cars) {
  double prediction_time = 1; // in s

  for ( const auto&v : cars ) {
    double ds = v[4] - ego.getPs();
    double dv  = ego.getSpeed() - v[2];
    double safe_distance = ego.getMinSafeDistance();

    if (ds >= 0 ) {
      // for the front car, increase the safe distance if ego is faster
      if ( dv > 0 ) { safe_distance += prediction_time*dv; }
      if ( ds < safe_distance ) { return true; }
    } else {
      // for the rear car, increase the distance if ego is slower
      if ( dv < 0 ) { safe_distance += -prediction_time*dv; }
      if ( std::abs(ds) < safe_distance ) { return true; }
    }
  }

  return false;
}
