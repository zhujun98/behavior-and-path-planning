//
// Created by jun on 7/24/17.
//
#include <iostream>

#include "vehicle.h"
#include "ego.h"
#include "ego_state_change_lane.h"
#include "map.h"
#include "utilities.h"
#include "ego_transition_state.h"


EgoStateChangeLane::EgoStateChangeLane() {
  transition_states_.push_back(EgoTransitionStateFactory::createState(CL_TO_FT));
}

EgoStateChangeLane::~EgoStateChangeLane() {}

void EgoStateChangeLane::onEnter(Ego& ego) {
  ego.truncatePath(5);
  planPath(ego);
  std::cout << "Enter state: *** CHANGE LANE *** from Lane-" << ego.getLaneID()
            << " to Lane-" << ego.getTargetLaneID() << std::endl;
}

void EgoStateChangeLane::onUpdate(Ego &ego) {
  ;
}

void EgoStateChangeLane::onExit(Ego& egop) {
  std::cout << "Exit state: *** CHANGE LANE *** " << std::endl;
}

void EgoStateChangeLane::planPath(Ego &ego) {
  auto state0_sd = getState0(ego);
  std::vector<double> state0_s = state0_sd.first;
  std::vector<double> state0_d = state0_sd.second;
  double ps0 = state0_s[0];
  double vs0 = state0_s[1];

  double ps1, vs1, as1;
  double pd1, vd1, ad1;

  vs1 = ego.getMaxSpeed();
  vd1 = 0;
  as1 = 0;
  ad1 = 0;
  double prediction_time = 3.0; // in s

  ps1 = ps0 + 0.5*(vs0 + vs1)*prediction_time;
  pd1 = (ego.getTargetLaneID() - 0.5) * ego.getMap()->getLaneWidth();

  std::vector<double> state1_s = {ps1, vs1, as1};
  std::vector<double> state1_d = {pd1, vd1, ad1};

  std::vector<double> coeff_s = jerkMinimizingTrajectory(state0_s, state1_s, prediction_time);
  std::vector<double> coeff_d = jerkMinimizingTrajectory(state0_d, state1_d, prediction_time);

  ego.extendPath(coeff_s, coeff_d, prediction_time);
}
