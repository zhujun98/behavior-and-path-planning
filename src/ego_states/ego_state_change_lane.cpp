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

void EgoStateChangeLane::planPath(Ego &ego) {
  auto state0 = getState0(ego);
  double ps0 = state0.first[0];
  double vs0 = state0.first[1];

  double ps1, vs1, as1;
  double pd1, vd1, ad1;

  auto vehicle_side_front = ego.getClosestVehicle(ego.getTargetLaneID(), 1);
  if ( vehicle_side_front.empty()) {
    vs1 = ego.getTargetSpeed();
  } else {
    vs1 = vehicle_side_front[1];
    if ( vs1 > ego.getTargetSpeed() ) { vs1 = ego.getTargetSpeed(); }
  }

  vd1 = 0;
  as1 = 0;
  ad1 = 0;
  double prediction_time = 3.0; // in s

  ps1 = ps0 + 0.5*(vs0 + vs1)*prediction_time;
  pd1 = (ego.getTargetLaneID() - 0.5) * ego.getMap()->getLaneWidth();

  std::vector<double> state1_s = {ps1, vs1, as1};
  std::vector<double> state1_d = {pd1, vd1, ad1};
  vehicle_state state1 = std::make_pair(state1_s, state1_d);

  PathPlanner planner(ego.getMaxSpeed(), ego.getMaxAcceleration(), ego.getMaxJerk());

  vehicle_trajectory new_path = planner.plan(state0, state1, prediction_time);

  ego.extendPath(new_path);
}
