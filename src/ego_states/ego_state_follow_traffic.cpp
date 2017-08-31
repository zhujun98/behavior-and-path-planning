//
// Created by jun on 7/27/17.
//
#include <iostream>
#include <vector>

#include "../ego.h"
#include "../map.h"
#include "ego_state_follow_traffic.h"
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
  planPath(ego);
}

void EgoStateFollowTraffic::onExit(Ego& ego) {
  std::cout << "Exit state: *** FOLLOW TRAFFIC ***" << std::endl;
}

void EgoStateFollowTraffic::planPath(Ego& ego) {
  ego.truncatePath(5);

  auto state0 = ego.getInitialState();
  double ps0 = state0.first[0];
  double vs0 = state0.first[1];

  // get the distance and the speed of the front car
  std::vector<double> front_vehicle = ego.getClosestVehicle(ego.getLaneID(), 1);
  double prediction_time = 1.0 - ego.getPathS()->size()*ego.getTimeStep();

  // Calculate the maximum reachable final speed. The maximum jerk is
  // not considered here. So this could be an over-estimation.
  double vs1 = vs0 + prediction_time*ego.getMaxAcceleration();
  if ( vs1 > ego.getTargetSpeed() ) { vs1 = ego.getTargetSpeed(); }

  // If there is vehicle in front of the ego car, then find the maximum
  // acceleration that can make a safe distance between the ego car
  // and the front car.
  if ( !front_vehicle.empty() ) {
    double ps_front = front_vehicle[0];
    double vs_front = front_vehicle[1];

    double acc = ego.getMaxAcceleration();
    int n_steps = 20;
    double acc_step = 2*ego.getMaxAcceleration()/n_steps;
    // The maximum deceleration is actually 3*max_acceleration_.
    // In practice, the number should be the maximum deceleration that
    // the car can physically achieve.
    for ( int i=0; i<=2*n_steps; ++i ) {
      vs1 = vs0 + prediction_time*acc;
      if ( vs1 > ego.getTargetSpeed() ) { vs1 = ego.getTargetSpeed(); }
      if ( vs1 < 0 ) { vs1 = 0; }

      double distance = ps_front - ps0 - ego.getMinSafeDistance(vs1) +
                        (vs_front - 0.5*(vs0 + vs1))*prediction_time;
      if ( distance > 0 ) { break; }

      acc -= acc_step;
    }
  }

  double ds1 = 0.5*(vs0 + vs1)*prediction_time;
  double pd1 = (ego.getLaneID() - 0.5) * ego.getMap()->getLaneWidth();

  PathPlanner planner(ego.getTargetSpeed(), ego.getMaxAcceleration(), ego.getMaxJerk());

  planner.setDsBoundary(ds1*0.9, ds1*1.1);
  planner.setVsBoundary(vs1*0.9, vs1*1.1);

  planner.setPdBoundary(pd1, pd1);
  planner.setVdBoundary(0, 0);

  vehicle_trajectory new_path = planner.plan(state0, prediction_time);

  ego.extendPath(new_path);
}