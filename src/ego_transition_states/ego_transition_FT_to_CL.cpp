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

bool EgoTransitionFTToCL::isOptimal(const Ego &ego, int direction) const {
  // Do nothing at the end or at the beginning of a lap.
  if ( ego.getPs() < 30 || ego.getMap()->getMaxS() - ego.getPs() < 30 ) {
    return false;
  }

  if ( direction != 1 && direction != -1 ) { return false; }

  int current_lane_id = ego.getLaneID();
  double MAX_DS = 1000;

  double prediction_time = 5.0;

  std::vector<double> ds_finals;  // ds at the prediction time
  // Get the distance to the front vehicles in different lanes at
  // the "prediction time"
  // Note: Lane ID starts from 1.
  for ( int i = 1; i <= ego.getMap()->getNoLanes(); ++i ) {
    auto front_vehicle = ego.getClosestVehicle(i, 1);
    double ds_final;
    if ( !front_vehicle.empty() ) {
      double ps = front_vehicle[0];
      double vs = front_vehicle[1];

      ds_final = ps - ego.getPs() + ( vs - ego.getVs() ) * prediction_time;
    } else {
      ds_final = MAX_DS; // a very large number if there no car on the lane
    }

    ds_finals.push_back(ds_final);
  }

  double max_distance_lane_id = current_lane_id;
  double max_ds_final = ds_finals[max_distance_lane_id - 1];

  for ( int i = 0; i < ds_finals.size(); ++i ) {
    // The second condition says always keep in the middle lane (yield
    // the overtake lane.
    if ( ds_finals[i] - max_ds_final > 15 ) {
      max_ds_final = ds_finals[i];
      max_distance_lane_id = i + 1; // lane id starts from 1
    } else if ( ds_finals[i] == MAX_DS && i == 1 ) {
      // keep in the middle lane if there are other empty lanes
      max_distance_lane_id = 2;
    }
  }

//   for debug
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
    return ( current_ds - next_ds < 40 );
  } else {
    return false;
  }
}

bool EgoTransitionFTToCL::planPath(Ego &ego, int direction) const {
  ego.truncatePath(5);
  auto state0 = ego.getInitialState();
  double vs0 = state0.first[1];
  double ps0 = state0.first[0];

  double t_total = 2.5 - ego.getPathS()->size()*ego.getTimeStep(); // in s
  // the estimated time that the car shifts to the boundary of the current lane
  double t_shift = 1.0;

  double vs1;
  double acc = ego.getMaxAcceleration();

  double acc_step = ego.getMaxAcceleration()/10;
  // "i << 15" says that the car is not allowed to perform overtaking
  // by dramatically reducing speed
  for ( int i = 0; i <= 15; ++i ) {
    if ( i != 0 ) { acc -= acc_step; }

    bool will_collision = false;

    // check collision with the vehicles in the same lane
    auto vehicles_in_lane = ego.getVehiclesInLane(ego.getLaneID());
    for ( const auto& v : vehicles_in_lane ) {
      double safe_maneuver_distance = 5; // in m

      double speed = v[2];
      double ps = v[4];

      vs1 = vs0 + t_shift*acc;
      if ( vs1 > ego.getTargetSpeed() ) { vs1 = ego.getTargetSpeed(); }
      if ( vs1 < 0 ) { vs1 = 0; }

      double distance_begin = ps - ps0;
      double distance_end = ps - ps0 + ( speed - 0.5*(vs0 + vs1) ) * t_shift;

      if ( distance_begin * distance_end > 0 ) {
        if ( std::abs(distance_begin) < safe_maneuver_distance ||
             std::abs(distance_end) < safe_maneuver_distance) {
          will_collision = true;
          break;
        }
      } else {
        will_collision = true;
        break;
      }
    }

    if ( will_collision ) { continue; }

    // check collision with the vehicles in the target lane
    auto vehicles_in_next_lane = ego.getVehiclesInLane(ego.getLaneID() + direction);
    for ( const auto& v : vehicles_in_next_lane ) {
      double safe_maneuver_distance = 10; // in m

      double speed = v[2];
      double ps = v[4];

      double vs_shift = vs0 + t_total*acc;
      if ( vs_shift > ego.getTargetSpeed() ) { vs_shift = ego.getTargetSpeed(); }
      if ( vs_shift < 0 ) { vs_shift = 0; }

      vs1 = vs0 + t_total*acc;
      if ( vs1 > ego.getTargetSpeed() ) { vs1 = ego.getTargetSpeed(); }
      if ( vs1 < 0 ) { vs1 = 0; }

      double distance_begin = ps - ps0;
      double distance_shift = ps - ps0 + ( speed - 0.5*(vs0 + vs_shift) ) * t_shift;
      double distance_end = ps - ps0 + ( speed - 0.5*(vs0 + vs1) ) * t_total;

      if ( distance_begin * distance_end > 0 && distance_begin * distance_shift > 0) {
        if ( std::abs(distance_begin) < safe_maneuver_distance ||
             std::abs(distance_shift) < safe_maneuver_distance ||
             std::abs(distance_end) < safe_maneuver_distance ) {
          will_collision = true;
          break;
        }
      } else {
        will_collision = true;
        break;
      }
    }

    if ( will_collision ) { continue; }

    ego.setTargetLaneID(ego.getLaneID() + direction);

    double ds1 = 0.5*(vs0 + vs1)*t_total;
    double pd1 = (ego.getTargetLaneID() - 0.5) * ego.getMap()->getLaneWidth();

    PathPlanner planner(ego.getTargetSpeed(), ego.getMaxAcceleration(), ego.getMaxJerk());

    planner.setDsBoundary(0.80*ds1, 1.0*ds1);
    planner.setVsBoundary(0.80*vs1, 1.0*vs1);
    planner.setAsBoundary(0, 0);

    planner.setPdBoundary(pd1, pd1);
    planner.setVdBoundary(0, 0);
    planner.setAdBoundary(0, 0);

    vehicle_trajectory new_path = planner.plan(state0, t_total);

    ego.extendPath(new_path);
    return true;
  }

  return false;
}
