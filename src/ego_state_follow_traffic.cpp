//
// Created by jun on 7/27/17.
//
#include <iostream>
#include <vector>

#include "ego.h"
#include "map.h"
#include "ego_state_follow_traffic.h"
#include "utilities.h"
#include "ego_transition_state.h"


EgoStateFollowTraffic::EgoStateFollowTraffic() {
  transition_states_.push_back(EgoTransitionStateFactory::createState(FT_TO_CS));
  transition_states_.push_back(EgoTransitionStateFactory::createState(FT_TO_CL));
}

EgoStateFollowTraffic::~EgoStateFollowTraffic() {}

void EgoStateFollowTraffic::onEnter(Ego& ego) {
  std::cout << "Enter state: *** FOLLOW TRAFFIC ***" << std::endl;
}

void EgoStateFollowTraffic::onUpdate(Ego &ego) {
  ego.truncatePath(5);
  planPath(ego);
}

void EgoStateFollowTraffic::onExit(Ego& ego) {
  std::cout << "Exit state: *** FOLLOW TRAFFIC ***" << std::endl;
}

void EgoStateFollowTraffic::planPath(Ego& ego) {
  auto state0_sd = getState0(ego);
  std::vector<double> state0_s = state0_sd.first;
  std::vector<double> state0_d = state0_sd.second;
  double ps0 = state0_s[0];
  double vs0 = state0_s[1];

  double ps1, vs1, as1;
  double pd1, vd1, ad1;

  // get the distance and the speed of the front car
  double ds_front = 1000;
  double vs_front = 1000;
  for ( auto &v : ego.getSurroundings()->center ) {
    double ds = v[4] - ego.getPs();
    if ( ds > 0 && ds < ds_front ) {
      ds_front = ds;
      vs_front = v[2];
    }
  }

  vs1 = vs_front;
  vd1 = 0;
  as1 = 0;
  ad1 = 0;

  double prediction_time = 2.0;

  ps1 = ps0 + ds_front + vs_front*prediction_time - ego.getMinSafeDistance();
  vs1 = 2*(ps1 - ps0) - vs0;
  if ( vs1 > ego.getMaxSpeed() ) { vs1 = ego.getMaxSpeed(); }

  pd1 = (ego.getLaneID() - 0.5) * ego.getMap()->getLaneWidth();

  std::vector<double> state1_s = {ps1, vs1, as1};
  std::vector<double> state1_d = {pd1, vd1, ad1};

  std::vector<double> coeff_s = jerkMinimizingTrajectory(state0_s, state1_s, prediction_time);
  std::vector<double> coeff_d = jerkMinimizingTrajectory(state0_d, state1_d, prediction_time);

  ego.extendPath(coeff_s, coeff_d, prediction_time);
}
