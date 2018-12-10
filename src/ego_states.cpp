//
// Created by jun on 7/24/17.
//
#include <iostream>

#include "ego.hpp"
#include "map.hpp"
#include "ego_states.hpp"
#include "ego_transition_states.hpp"


/*
 * EgoState class
 */

EgoState::EgoState() : timer_(0.0) {}

EgoState::~EgoState() = default;

EgoState* EgoState::checkTransition(Ego &ego) {
  ++timer_;
  // avoid frequently switching between states
  if ( timer_ < 5 ) return nullptr;

  for (const auto& v : transition_states_)
    if ( v->isValid(ego) ) { return v->getNextState(ego); }

  return nullptr;
}


/*
 * EgoStateFactory class
 */

EgoStateFactory::EgoStateFactory() = default;

EgoStateFactory::~EgoStateFactory() = default;

EgoState* EgoStateFactory::createState(States name) {
  switch(name) {
    case CLR:
      return new EgoStateChangeLaneRight;
    case CLL:
      return new EgoStateChangeLaneLeft;
    case FT:
      return new EgoStateFollowTraffic;
    default:
      throw std::invalid_argument("Unknown state!");
  }
}


/*
 * EgoStateFollowTraffic class
 */

EgoStateFollowTraffic::EgoStateFollowTraffic() {
  transition_states_.push_back(EgoTransitionStateFactory::createState(FT_TO_CLR));
  transition_states_.push_back(EgoTransitionStateFactory::createState(FT_TO_CLL));
}

EgoStateFollowTraffic::~EgoStateFollowTraffic() = default;

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
//  ego.truncatePath(5);
//
//  auto state0 = ego.getCurrentState();
//  double ps0 = state0.first[0];
//  double vs0 = state0.first[1];
//
//  // get the distance and the speed of the front car
//  std::vector<double> front_vehicle = ego.getClosestVehicle(ego.getCurrentLaneId(), 1);
//  double prediction_time = 1.0 - ego.getPathS()->size()*ego.getTimeStep();
//
//  // Calculate the maximum reachable final speed. The maximum jerk is
//  // not considered here. So this could be an over-estimation.
//  double vs1 = vs0 + prediction_time*ego.getMaxAcceleration();
//  if ( vs1 > ego.getTargetSpeed() ) { vs1 = ego.getTargetSpeed(); }
//
//  // If there is vehicle in front of the ego car, then find the maximum
//  // acceleration that can make a safe distance between the ego car
//  // and the front car.
//  if ( !front_vehicle.empty() ) {
//    double ps_front = front_vehicle[0];
//    double vs_front = front_vehicle[1];
//
//    double acc = ego.getMaxAcceleration();
//    int n_steps = 20;
//    double acc_step = 2*ego.getMaxAcceleration()/n_steps;
//    // The maximum deceleration is actually 3*max_acceleration_.
//    // In practice, the number should be the maximum deceleration that
//    // the car can physically achieve.
//    for ( int i=0; i<=2*n_steps; ++i ) {
//      vs1 = vs0 + prediction_time*acc;
//      if ( vs1 > ego.getTargetSpeed() ) { vs1 = ego.getTargetSpeed(); }
//      if ( vs1 < 0 ) { vs1 = 0; }
//
//      double distance = ps_front - ps0 - ego.getMinSafeDistance(vs1) +
//                        (vs_front - 0.5*(vs0 + vs1))*prediction_time;
//      if ( distance > 0 ) { break; }
//
//      acc -= acc_step;
//    }
//  }
//
//  double ds1 = 0.5*(vs0 + vs1)*prediction_time;
//  double pd1 = ego.getLaneCenter();
//
//  PathPlanner planner(ego.getTargetSpeed(), ego.getMaxAcceleration(), ego.getMaxJerk());
//
//  planner.setDsBoundary(ds1*0.8, ds1*1.2);
//  planner.setVsBoundary(vs1*0.8, vs1*1.2);
//
//  planner.setPdBoundary(pd1, pd1);
//
//  trajectory new_path = planner.plan(state0, prediction_time);
//
//  ego.extendPath(new_path);
}


/*
 * EgoStateChangeLaneLeft class
 */

EgoStateChangeLaneLeft::EgoStateChangeLaneLeft() {
  transition_states_.push_back(EgoTransitionStateFactory::createState(CL_TO_FT));
}

EgoStateChangeLaneLeft::~EgoStateChangeLaneLeft() = default;

void EgoStateChangeLaneLeft::onEnter(Ego& ego) {
  std::cout << "Enter state: *** CHANGE TO THE LEFT LANE *** from Lane-" << ego.getCurrentLaneId()
            << " to Lane-" << ego.getTargetLaneId() << std::endl;
}

void EgoStateChangeLaneRight::onUpdate(Ego& ego) {}

void EgoStateChangeLaneLeft::onExit(Ego& egop) {
  std::cout << "Exit state: *** CHANGE TO THE LEFT LANE *** " << std::endl;
}

/*
 * EgoStateChangeLaneRight class
 */

EgoStateChangeLaneRight::EgoStateChangeLaneRight() {
  transition_states_.push_back(EgoTransitionStateFactory::createState(CL_TO_FT));
}

EgoStateChangeLaneRight::~EgoStateChangeLaneRight() = default;

void EgoStateChangeLaneRight::onEnter(Ego& ego) {
  std::cout << "Enter state: *** CHANGE TO THE RIGHT LANE *** from Lane-" << ego.getCurrentLaneId()
            << " to Lane-" << ego.getTargetLaneId() << std::endl;
}

void EgoStateChangeLaneLeft::onUpdate(Ego& ego) {}

void EgoStateChangeLaneRight::onExit(Ego& egop) {
  std::cout << "Exit state: *** CHANGE TO THE RIGHT LANE *** " << std::endl;
}