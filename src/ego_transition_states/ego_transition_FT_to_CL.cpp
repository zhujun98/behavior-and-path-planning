//
// Created by jun on 7/28/17.
//
#include <cmath>

#include "ego_transition_FT_to_CL.h"
#include "../map.h"
#include "../ego.h"
#include "../ego_states/ego_state.h"


EgoTransitionFTToCL::EgoTransitionFTToCL() {}

EgoTransitionFTToCL::~EgoTransitionFTToCL() {}

bool EgoTransitionFTToCL::willCollision(const Ego& ego, int direction) const {
  if ( direction != 1 && direction != -1 ) { return true; }

  double prediction_time = 0.5; // in s
  double safe_maneuver_distance = 10; // in m

  std::vector<double> front_vehicle = ego.getClosestVehicle(ego.getLaneID(), 1);
  std::vector<double> side_front_vehicle = ego.getClosestVehicle(ego.getLaneID() + direction, 1);
  std::vector<double> side_rear_vehicle = ego.getClosestVehicle(ego.getLaneID() + direction, -1);

  // estimated distance after the prediction time
  double distance_front = 1000; // a large number
  if ( !front_vehicle.empty() ) {
    double ps_front = front_vehicle[0];
    double vs_front = front_vehicle[1];
    distance_front = ps_front - ego.getPs() + ( vs_front - ego.getVs() ) * prediction_time;
  }

  double distance_side_front = 1000; // a large number
  if ( !side_front_vehicle.empty() ) {
    double ps_front = side_front_vehicle[0];
    double vs_front = side_front_vehicle[1];
    distance_side_front = ps_front - ego.getPs() + ( vs_front - ego.getVs() ) * prediction_time;
  }

  double distance_side_rear = 1000; // a large number
  if ( !side_rear_vehicle.empty() ) {
    double ps_rear = side_rear_vehicle[0];
    double vs_rear = side_rear_vehicle[1];
    distance_side_rear = ego.getPs() - ps_rear + ( ego.getVs() - vs_rear ) * prediction_time;
  }

  return ( (std::abs(distance_front) < safe_maneuver_distance) ||
           (std::abs(distance_side_front) < safe_maneuver_distance) ||
           (std::abs(distance_side_rear) < safe_maneuver_distance) );
}


bool EgoTransitionFTToCL::isOptimal(const Ego &ego, int direction) const {
  if ( direction != 1 && direction != -1 ) { return false; }

  // A small prediction time here unless we know the front vehicle
  // dynamics very well.
  double prediction_time = 1.0;

  std::vector<double> ds_finals;
  // lane ID starts from 1
  for ( int i = 1; i <= ego.getMap()->getNoLanes(); ++i ) {
    auto front_vehicle = ego.getClosestVehicle(i, 1);
    double ds_final;
    if ( !front_vehicle.empty() ) {
      double ps = front_vehicle[0];
      double vs = front_vehicle[1];

      ds_final = ps - ego.getPs() + ( vs - ego.getVs() ) * prediction_time;
    } else {
      ds_final = 1000; // a very large number if there no car on the lane
    }

    ds_finals.push_back(ds_final);
  }

  double max_distance_lane_id = ego.getLaneID();
  double max_ps_final = ds_finals[max_distance_lane_id - 1];
  // be careful since land ID starts from 1
  for ( int i = 0; i < ds_finals.size(); ++i ) {
    // Shift the lane unless there is some gap.
    // Also ensure that the current lane is the best when having the
    // same cost.
    if ( ds_finals[i] > max_ps_final + 15 ) {
      max_ps_final = ds_finals[i];
      max_distance_lane_id = i + 1;
    }
  }

  if ( ego.getTicker() % 20  == 0 ) {
    std::cout << "Current lane ID: " << ego.getLaneID()
              << ", Best land ID: " << max_distance_lane_id << std::endl;
  }

  // if the lane which has the most space is on the same direction as the
  // current planned lane-changing direction. Then do it!
  return ( direction*(max_distance_lane_id - ego.getLaneID()) >= 1 );
}