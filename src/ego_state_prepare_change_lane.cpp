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


EgoStatePrepareChangeLane::EgoStatePrepareChangeLane() {
  is_ready_ = true;
}

EgoStatePrepareChangeLane::~EgoStatePrepareChangeLane() {}

void EgoStatePrepareChangeLane::onEnter(Ego& ego) {
  std::cout << "Enter state: *** PREPARE CHANGE LANE ***" << std::endl;
}

EgoState* EgoStatePrepareChangeLane::onUpdate(Ego& ego,
                                              const std::vector<std::vector<double>>& sensor_fusion,
                                              const Map& map) {
  int current_lane_id = ego.getLaneID();
  if ( current_lane_id > 1 ) {
    target_lane_id_ = current_lane_id - 1;
  } else if ( current_lane_id == 1 ){
    target_lane_id_ = current_lane_id + 1;
  }

  if ( is_ready_ ) {
    ego.truncatePath(5);
    planPath(ego, map);
    return new EgoStateChangeLane(target_lane_id_);
  } else {
    // check collision
    return nullptr;
  }
}

void EgoStatePrepareChangeLane::onExit(Ego& ego) {
  std::cout << "Exist state: *** PREPARE CHANGE LANE ***" << std::endl;
}

void EgoStatePrepareChangeLane::planPath(Ego& ego, const Map& map) {

  double ps0, vs0, as0;
  double pd0, vd0, ad0;

  double ps1, vs1, as1;
  double pd1, vd1, ad1;

  if ( ego.getPath().first.empty() ) {
    ps0 = ego.getPs();
    pd0 = ego.getPd();
  } else {
    ps0 = *std::next(ego.getPath().first.end(), -1);
    pd0 = *std::next(ego.getPath().second.end(), -1);
  }

  vs0 = ego.getMaxSpeed();
  vd0 = 0;
  as0 = 0;
  ad0 = 0;
  vs1 = vs0;
  vd1 = 0;
  as1 = 0;
  ad1 = 0;
  double duration = ego.getTimeStep() * ego.getPredictionPts();

  ps1 = ps0 + vs0*duration;
  pd1 = (target_lane_id_ - 0.5) * map.getLaneWidth();

  std::vector<double> state0_s = {ps0, vs0, as0};
  std::vector<double> state0_d = {pd0, vd0, ad0};
  std::vector<double> state1_s = {ps1, vs1, as1};
  std::vector<double> state1_d = {pd1, vd1, ad1};

  std::vector<double> coeff_s = jerkMinimizingTrajectory(state0_s, state1_s, duration);
  std::vector<double> coeff_d = jerkMinimizingTrajectory(state0_d, state1_d, duration);

  ego.extendPath(coeff_s, coeff_d);
}
