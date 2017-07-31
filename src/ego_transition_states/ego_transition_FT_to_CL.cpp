//
// Created by jun on 7/28/17.
//
#include <cmath>

#include "ego_transition_FT_to_CL.h"
#include "../map.h"
#include "../ego.h"
#include "../ego_states/ego_state.h"
#include "../utilities.h"


EgoTransitionFTToCL::EgoTransitionFTToCL() {}

EgoTransitionFTToCL::~EgoTransitionFTToCL() {}

bool EgoTransitionFTToCL::willCollision(const Ego& ego, int direction) const {
  if ( direction != 1 && direction != -1 ) { return true; }

  // check collision with the vehicles in the same lane
  auto vehicles_in_lane = ego.getVehiclesInLane(ego.getLaneID());
  for ( const auto& v : vehicles_in_lane ) {
    double safe_maneuver_distance = 5; // in m

    double t0 = 0;
    double t1 = 1.0;
    double speed = v[2];
    double ps = v[4];

    double distance_start = ps - ego.getPs() + ( speed - ego.getVs() ) * t0;
    double distance_end = ps - ego.getPs() + ( speed - ego.getVs() ) * t1;

    if ( distance_start * distance_end > 0 ) {
      if ( std::abs(distance_start) < safe_maneuver_distance ||
           std::abs(distance_end) < safe_maneuver_distance ) { return true; }
    } else {
      return true;
    }
  }

  // check collision with the vehicles in the target lane
  auto vehicles_in_next_lane = ego.getVehiclesInLane(ego.getLaneID() + direction);
  for ( const auto& v : vehicles_in_next_lane ) {
    double safe_maneuver_distance = 10; // in m

    double t0 = 0;
    double t1 = 3.0;
    double speed = v[2];
    double ps = v[4];

    double distance_start = ps - ego.getPs() + ( speed - ego.getVs() ) * t0;
    double distance_end = ps - ego.getPs() + ( speed - ego.getVs() ) * t1;

    if ( distance_start * distance_end > 0 ) {
      if ( std::abs(distance_start) < safe_maneuver_distance ||
           std::abs(distance_end) < safe_maneuver_distance ) { return true; }
    } else {
      return true;
    }
  }

  return false;

}

bool EgoTransitionFTToCL::isOptimal(const Ego &ego, int direction) const {
  if ( direction != 1 && direction != -1 ) { return false; }

  int current_lane_id = ego.getLaneID();
  // A small prediction time here unless we know the front vehicle
  // dynamics very well.
  double prediction_time = 1.0;

  std::vector<double> ds_finals;  // ds at the prediction time
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

  double max_distance_lane_id = current_lane_id;
  double max_ds_final = ds_finals[max_distance_lane_id - 1];
  // be careful since land ID starts from 1
  for ( int i = 0; i < ds_finals.size(); ++i ) {
    // The second condition says always keep in the middle lane (yield
    // the overtake lane.
    if (( ds_finals[i] > max_ds_final ) ||
       ( std::abs(ds_finals[i] - max_ds_final) < 5 && i == 1 )) {
      max_ds_final = ds_finals[i];
      max_distance_lane_id = i + 1;
    }
  }

  // for debug
  if ( ego.getTicker() % 10  == 0 ) {
    std::cout << "Current lane ID: " << current_lane_id
              << ", Best land ID: " << max_distance_lane_id << ": ";
    print1DContainer(ds_finals);
  }

  if ( max_distance_lane_id - current_lane_id == direction ) {
    return true;
  } else if ( direction*(max_distance_lane_id - current_lane_id) > 1 ) {
    double current_ds = ds_finals[current_lane_id - 1];
    double next_ds = ds_finals[current_lane_id - 1 + direction ];

    // if there is not enough room to change lane if the car keeps going
    return ( current_ds - next_ds < 30 );
  } else {
    return false;
  }
}