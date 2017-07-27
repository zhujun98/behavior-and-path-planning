//
// Created by jun on 7/25/17.
//

#ifndef PATH_PLANNING_EGO_STATE_PREPARE_CHANGE_LANE_H
#define PATH_PLANNING_EGO_STATE_PREPARE_CHANGE_LANE_H

#include <vector>

#include "ego_state.h"

class Ego;
class Map;


class EgoStatePrepareChangeLane : public EgoState {
private:

  int target_lane_id_;

  void planPath(Ego& ego);

  bool checkCollision(const Ego& ego);

  bool checkSideCollision(const Ego& ego, std::vector<std::vector<double>> cars);

public:
  //
  //
  //
  EgoStatePrepareChangeLane();

  //
  //
  //
  ~EgoStatePrepareChangeLane();

  void onEnter(Ego& ego);

  EgoState* onUpdate(Ego& ego);

  void onExit(Ego& ego);

};


#endif //PATH_PLANNING_EGO_STATE_PREPARE_CHANGE_LANE_H
