//
// Created by jun on 7/28/17.
//
#include <iostream>
#include <cmath>

#include "ego.hpp"
#include "map.hpp"
#include "ego_states.hpp"
#include "ego_transition_states.hpp"


EgoTransitionState* EgoTransitionStateFactory::createState(TransitionStates name) {
  switch (name) {
    case CL_TO_FT:
      static EgoTransitionCLToFT cl_to_ft;
      return &cl_to_ft;
    case FT_TO_CLR:
      static EgoTransitionFTToCLR ft_to_clr;
      return &ft_to_clr;
    case FT_TO_CLL:
      static EgoTransitionFTToCLL ft_to_cll;
      return &ft_to_cll;
    default:
      throw std::invalid_argument("Unknown transition state!");
  }
}


EgoTransitionFTToCLR::EgoTransitionFTToCLR() = default;

EgoTransitionFTToCLR::~EgoTransitionFTToCLR() = default;

EgoState* EgoTransitionFTToCLR::getNextState(Ego& ego) const {
  return EgoStateFactory::createState(CLR);
}

bool EgoTransitionFTToCLR::isValid(Ego &ego) const {
  return ( isOptimal(ego, 1) && planPath(ego, 1) );
}


EgoTransitionFTToCLL::EgoTransitionFTToCLL() = default;

EgoTransitionFTToCLL::~EgoTransitionFTToCLL() = default;

EgoState* EgoTransitionFTToCLL::getNextState(Ego& ego) const {
  return EgoStateFactory::createState(CLL);
}

bool EgoTransitionFTToCLL::isValid(Ego &ego) const {
  return ( isOptimal(ego, -1) && planPath(ego, -1) );
}


EgoTransitionFTToCL::EgoTransitionFTToCL() = default;

EgoTransitionFTToCL::~EgoTransitionFTToCL() = default;

bool EgoTransitionFTToCL::isOptimal(const Ego &ego, int direction) const {
  if (ego.isAroundOrigin()) return false;

  if ( direction != 1 && direction != -1 ) { return false; }

  int current_lane_id = ego.getCurrentLaneId();
  const double kMaxDist = 1000000;

  double prediction_time = 5.0;

  std::vector<double> ds_finals;  // ds at the prediction time
  // Get the distance to the front vehicles in different lanes at
  // the "prediction time"
  // Note: Lane ID starts from 1.
//  for ( int i = 1; i <= ego.getMap()->nLanes(); ++i ) {
//    auto front_vehicle = ego.getClosestVehicle(i, 1);
//    double ds_final;
//    if ( !front_vehicle.empty() ) {
//      double ps = front_vehicle[0];
//      double vs = front_vehicle[1];
//
//      ds_final = ps - ego.getPs() + ( vs - ego.getVs() ) * prediction_time;
//    } else {
//      ds_final = kMaxDist; // a very large number if there is no car on the lane
//    }
//
//    ds_finals.push_back(ds_final);
//  }

//  double max_distance_lane_id = current_lane_id;
//  double max_ds_final = ds_finals[max_distance_lane_id - 1];
//
//  for ( int i = 0; i < ds_finals.size(); ++i ) {
//    // The second condition says always keep in the middle lane (yield
//    // the overtake lane.
//    if ( ds_finals[i] - max_ds_final > 15 ) {
//      max_ds_final = ds_finals[i];
//      max_distance_lane_id = i + 1; // lane id starts from 1
//    }
//  }
//
//  if ( max_distance_lane_id - current_lane_id == direction ) {
//    return true;
//  }
//
//  if ( direction*(max_distance_lane_id - current_lane_id) > 1 ) {
//    double current_ds = ds_finals[current_lane_id - 1];
//    double next_ds = ds_finals[current_lane_id - 1 + direction ];
//
//    // if there is not enough room to change lane if the car keeps going
//    return ( current_ds - next_ds < 40 );
//  }

  return false;
}

bool EgoTransitionFTToCL::planPath(Ego &ego, int direction) const {
  ego.truncatePath(5);

//  auto state0 = ego.getCurrentState();
//  double vs0 = state0.first[1];
//  double ps0 = state0.first[0];
//
//  double t_total = 2.5 - ego.getPathS()->size()*ego.getTimeStep(); // in s
//  // the estimated time that the car shifts to the boundary of the current lane
//  double t_shift = 1.0;
//
//  double vs1;
//  double acc = ego.getMaxAcceleration();
//
//  double acc_step = ego.getMaxAcceleration()/10;
//  // "i << 15" says that the car is not allowed to perform overtaking
//  // at a very low speed
//  for ( int i = 0; i <= 15; ++i ) {
//    if ( i != 0 ) { acc -= acc_step; }
//
//    bool will_collision = false;
//
//    // check collision with the vehicles in the same lane
//    auto vehicles_in_lane = ego.getVehiclesInLane(ego.getCurrentLaneId());
//    for ( const auto& v : vehicles_in_lane ) {
//      double safe_maneuver_distance = 5; // in m
//
//      double speed = v[2];
//      double ps = v[4];
//
//      vs1 = vs0 + t_shift*acc;
//      if ( vs1 > ego.getTargetSpeed() ) { vs1 = ego.getTargetSpeed(); }
//      if ( vs1 < 0 ) { vs1 = 0; }
//
//      double distance_begin = ps - ps0;
//      double distance_end = ps - ps0 + ( speed - 0.5*(vs0 + vs1) ) * t_shift;
//
//      if ( distance_begin * distance_end > 0 ) {
//        if ( std::abs(distance_begin) < safe_maneuver_distance ||
//             std::abs(distance_end) < safe_maneuver_distance) {
//          will_collision = true;
//          break;
//        }
//      } else {
//        will_collision = true;
//        break;
//      }
//    }
//
//    if ( will_collision ) { continue; }
//
//    // check collision with the vehicles in the target lane
//    auto vehicles_in_next_lane = ego.getVehiclesInLane(ego.getCurrentLaneId() + direction);
//    for ( const auto& v : vehicles_in_next_lane ) {
//      double safe_maneuver_distance = 10; // in m
//
//      double speed = v[2];
//      double ps = v[4];
//
//      double vs_shift = vs0 + t_total*acc;
//      if ( vs_shift > ego.getTargetSpeed() ) { vs_shift = ego.getTargetSpeed(); }
//      if ( vs_shift < 0 ) { vs_shift = 0; }
//
//      vs1 = vs0 + t_total*acc;
//      if ( vs1 > ego.getTargetSpeed() ) { vs1 = ego.getTargetSpeed(); }
//      if ( vs1 < 0 ) { vs1 = 0; }
//
//      double distance_begin = ps - ps0;
//      double distance_shift = ps - ps0 + ( speed - 0.5*(vs0 + vs_shift) ) * t_shift;
//      double distance_end = ps - ps0 + ( speed - 0.5*(vs0 + vs1) ) * t_total;
//
//      if ( distance_begin * distance_end > 0 && distance_begin * distance_shift > 0) {
//        if ( std::abs(distance_begin) < safe_maneuver_distance ||
//             std::abs(distance_shift) < safe_maneuver_distance ||
//             std::abs(distance_end) < safe_maneuver_distance ) {
//          will_collision = true;
//          break;
//        }
//      } else {
//        will_collision = true;
//        break;
//      }
//    }
//
//    if ( will_collision ) { continue; }
//
//    ego.setTargetLaneId(ego.getCurrentLaneId() + direction);
//
//    double ds1 = 0.5*(vs0 + vs1)*t_total;
//    double pd1 = ego.getTargetLaneCenter();
//
//    PathPlanner planner(ego.getTargetSpeed(), ego.getMaxAcceleration(), ego.getMaxJerk());
//
//    planner.setDsBoundary(ds1*0.9, ds1*1.1);
//    planner.setVsBoundary(vs1*0.9, vs1*1.1);
//
//    planner.setPdBoundary(pd1, pd1);
//    planner.setVdBoundary(0, 0);
//
//    trajectory new_path = planner.plan(state0, t_total);
//
//    ego.extendPath(new_path);
//    return true;
//  }

  return false;
}


EgoTransitionCLToFT::EgoTransitionCLToFT() = default;

EgoTransitionCLToFT::~EgoTransitionCLToFT() = default;

EgoState* EgoTransitionCLToFT::getNextState(Ego& ego) const {
  return EgoStateFactory::createState(FT);
}

bool EgoTransitionCLToFT::isValid(Ego &ego) const {
  return ( ego.getTargetLaneId() == ego.getCurrentLaneId() );
}