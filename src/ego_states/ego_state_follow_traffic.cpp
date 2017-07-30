//
// Created by jun on 7/27/17.
//
#include <iostream>
#include <vector>

#include "../ego.h"
#include "../map.h"
#include "ego_state_follow_traffic.h"
#include "../utilities.h"
#include "../ego_transition_states/ego_transition_state.h"


EgoStateFollowTraffic::EgoStateFollowTraffic() {
  transition_states_.push_back(EgoTransitionStateFactory::createState(FT_TO_CS));
  transition_states_.push_back(EgoTransitionStateFactory::createState(FT_TO_CLR));
  transition_states_.push_back(EgoTransitionStateFactory::createState(FT_TO_CLL));
}

EgoStateFollowTraffic::~EgoStateFollowTraffic() {}

void EgoStateFollowTraffic::onEnter(Ego& ego) {
  std::cout << "Enter state: *** FOLLOW TRAFFIC ***" << std::endl;
}

void EgoStateFollowTraffic::onUpdate(Ego &ego) {
  ego.truncatePath(20);
  planPath(ego);
}

void EgoStateFollowTraffic::onExit(Ego& ego) {
  std::cout << "Exit state: *** FOLLOW TRAFFIC ***" << std::endl;
}

void EgoStateFollowTraffic::planPath(Ego& ego) {
  auto state0 = getState0(ego);
  double ps0 = state0.first[0];
  double vs0 = state0.first[1];

  // get the distance and the speed of the front car
  std::vector<double> front_vehicle = ego.getClosestVehicle(ego.getLaneID(), 1);
  double prediction_time = 2.0 - ego.getPathS()->size()*ego.getTimeStep();

  double vs1 = ego.getTargetSpeed();
  if ( !front_vehicle.empty() ) {
    double ps_front = front_vehicle[0];
    double vs_front = front_vehicle[1];
    double ds1_safe = ps_front - ps0 + vs_front*prediction_time - ego.getMinSafeDistance();
    double vs1_safe = 2*ds1_safe / prediction_time - vs0;
    if ( vs1_safe < ego.getTargetSpeed() ) { vs1 = vs1_safe; }
  }
  double ds1 = 0.5*(vs0 + vs1)*prediction_time;

  double pd1 = (ego.getLaneID() - 0.5) * ego.getMap()->getLaneWidth();

  PathPlanner planner(ego.getTargetSpeed(), ego.getMaxAcceleration(), ego.getMaxJerk());

  planner.setDsBoundary(0.9*ds1, ds1);
  planner.setVsBoundary(0.9*vs1, vs1);
  planner.setAsBoundary(0, 0);
  planner.setPdBoundary(pd1, pd1);
  planner.setVdBoundary(0, 0);
  planner.setAdBoundary(0, 0);

  vehicle_trajectory new_path = planner.plan(state0, prediction_time);

  ego.extendPath(new_path);
}
