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
  double vs0 = state0.first[1];

  auto vehicle_side_front = ego.getClosestVehicle(ego.getTargetLaneID(), 1);

  // extend the current path
  double prediction_time = 4.0 - ego.getPathS()->size()*ego.getTimeStep(); // in s

  double vs1 = vs0 + ego.getMaxAcceleration()*prediction_time;
  if ( vs1 > ego.getTargetSpeed() ) { vs1 = ego.getTargetSpeed(); }
  double ds1 = 0.5*(vs0 + vs1)*prediction_time;

  double pd1 = (ego.getTargetLaneID() - 0.5) * ego.getMap()->getLaneWidth();

  PathPlanner planner(ego.getTargetSpeed(), ego.getMaxAcceleration(), ego.getMaxJerk());

  planner.setDsBoundary(0.95*ds1, 1.05*ds1);
  planner.setVsBoundary(0.95*vs1, 1.05*vs1);
  planner.setAsBoundary(0, 0);

  planner.setPdBoundary(pd1, pd1);
  planner.setVdBoundary(0, 0);
  planner.setAdBoundary(0, 0);

  vehicle_trajectory new_path = planner.plan(state0, prediction_time);

  ego.extendPath(new_path);
}
