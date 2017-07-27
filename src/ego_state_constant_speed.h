//
// Created by jun on 7/24/17.
//

#ifndef PATH_PLANNING_EGO_STATE_CONSTANT_SPEED_H
#define PATH_PLANNING_EGO_STATE_CONSTANT_SPEED_H

#include <vector>

#include "ego_state.h"

class Ego;
class Map;


class EgoStateConstantSpeed : public EgoState {
private:

  void planPath(Ego& ego, const Map& map);

  bool checkFrontCollision(const Ego& ego, const Map& map);

public:
  //
  // constructor
  //
  EgoStateConstantSpeed();

  //
  // destructor
  //
  ~EgoStateConstantSpeed();

  void onEnter(Ego& ego);

  EgoState* onUpdate(Ego& ego, const Map& map);

  void onExit(Ego& ego);
};


#endif //PATH_PLANNING_EGO_STATE_CONSTANT_SPEED_H
