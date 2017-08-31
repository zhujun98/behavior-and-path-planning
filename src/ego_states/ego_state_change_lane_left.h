//
// Created by jun on 7/29/17.
//
#ifndef PATH_PLANNING_EGO_STATE_CHANGE_LANE_LEFT_H
#define PATH_PLANNING_EGO_STATE_CHANGE_LANE_LEFT_H

#include "ego_state_change_lane.h"


class EgoStateChangeLaneLeft : public EgoStateChangeLane {
public:

  // constructor
  EgoStateChangeLaneLeft();

  // destructor
  ~EgoStateChangeLaneLeft() override;

  void onEnter(Ego& ego) override;

  void onExit(Ego& ego) override;
};


#endif //PATH_PLANNING_EGO_STATE_CHANGE_LANE_LEFT_H