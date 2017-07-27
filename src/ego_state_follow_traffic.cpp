//
// Created by jun on 7/27/17.
//
#include <iostream>
#include <vector>

#include "ego.h"
#include "map.h"
#include "ego_state_follow_traffic.h"
#include "ego_state_prepare_change_lane.h"
#include "utilities.h"


EgoStateFollowTraffic::EgoStateFollowTraffic() {}

EgoStateFollowTraffic::~EgoStateFollowTraffic() {}

void EgoStateFollowTraffic::onEnter(Ego& ego) {
  std::cout << "Enter state: *** FOLLOW TRAFFIC ***" << std::endl;
}

EgoState* EgoStateFollowTraffic::onUpdate(Ego& ego) {
  ego.truncatePath(5);
  planPath(ego);

  return new EgoStatePrepareChangeLane();
}

void EgoStateFollowTraffic::onExit(Ego& ego) {
  std::cout << "Exit state: *** FOLLOW TRAFFIC ***" << std::endl;
}

void EgoStateFollowTraffic::planPath(Ego& ego) {
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

  double duration = 1.0;

  vs0 = vs_front + (ds_front - ego.getMinSafeDistance())/duration;
  if ( vs0 > ego.getMaxSpeed() ) { vs0 = ego.getMaxSpeed(); }
  vd0 = 0;
  as0 = 0;
  ad0 = 0;

  vs1 = vs0;
  vd1 = 0;
  as1 = 0;
  ad1 = 0;

  ps1 = ps0 + 0.5*(vs0 + vs1)*duration;
  pd1 = (ego.getLaneID() - 0.5) * ego.getMap()->getLaneWidth();

  std::vector<double> state0_s = {ps0, vs0, as0};
  std::vector<double> state0_d = {pd0, vd0, ad0};
  std::vector<double> state1_s = {ps1, vs1, as1};
  std::vector<double> state1_d = {pd1, vd1, ad1};

  std::vector<double> coeff_s = jerkMinimizingTrajectory(state0_s, state1_s, duration);
  std::vector<double> coeff_d = jerkMinimizingTrajectory(state0_d, state1_d, duration);

  ego.extendPath(coeff_s, coeff_d);
}

bool EgoStateFollowTraffic::checkCollision(const Ego& ego) {
  ;
}