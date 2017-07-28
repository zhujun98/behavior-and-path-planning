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
  vs1 = ego.getMaxSpeed();
  vd1 = 0;
  as1 = 0;
  ad1 = 0;
  double duration = ego.getTimeStep() * ego.getPredictionPts();

  ps1 = ps0 + 0.5*(vs0 + vs1)*duration;
  pd1 = (ego.getTargetLaneID() - 0.5) * ego.getMap()->getLaneWidth();

  std::vector<double> state0_s = {ps0, vs0, as0};
  std::vector<double> state0_d = {pd0, vd0, ad0};
  std::vector<double> state1_s = {ps1, vs1, as1};
  std::vector<double> state1_d = {pd1, vd1, ad1};

  std::vector<double> coeff_s = jerkMinimizingTrajectory(state0_s, state1_s, duration);
  std::vector<double> coeff_d = jerkMinimizingTrajectory(state0_d, state1_d, duration);

  ego.extendPath(coeff_s, coeff_d);
}
