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
  bool is_ready_;

  void planPath(Ego& ego, const Map& map);

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

  EgoState* onUpdate(Ego& ego,
                     const std::vector<std::vector<double>>& sensor_fusion,
                     const Map& map);

  void onExit(Ego& ego);

};


#endif //PATH_PLANNING_EGO_STATE_PREPARE_CHANGE_LANE_H
