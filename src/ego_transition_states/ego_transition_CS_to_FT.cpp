//
// Created by jun on 7/28/17.
//

#include "ego_transition_CS_to_FT.h"
#include "../ego.h"
#include "../ego_states/ego_state.h"


EgoTransitionCSToFT::EgoTransitionCSToFT() {}

EgoTransitionCSToFT::~EgoTransitionCSToFT() {}

EgoState* EgoTransitionCSToFT::getNextState(Ego& ego) const {
  return EgoStateFactory::createState(FT);
}

bool EgoTransitionCSToFT::isValid(Ego &ego) const {
  // It requires a longer prediction time here to make the car
  // have enough space to adjust its speed in follow traffic state.
  double prediction_time = 5.0; // in s

  std::vector<double> front_vehicle = ego.getClosestVehicle(ego.getLaneID(), 1);
  if ( front_vehicle.empty() ) { return false; }
  double ps_front = front_vehicle[0];
  double vs_front = front_vehicle[1];

  double ps_ego = ego.getPs();
  double vs_ego = ego.getTargetSpeed();

  return ( (ps_front - ps_ego + (vs_front - vs_ego)*prediction_time) <
            ego.getMinSafeDistance() );
}