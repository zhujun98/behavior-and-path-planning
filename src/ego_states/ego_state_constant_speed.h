//
// Created by jun on 7/24/17.
//

#ifndef PATH_PLANNING_EGO_STATE_CONSTANT_SPEED_H
#define PATH_PLANNING_EGO_STATE_CONSTANT_SPEED_H

#include <vector>


class Ego;
class EgoState;


class EgoStateConstantSpeed : public EgoState {
private:

  void planPath(Ego& ego);

public:

  // constructor
  EgoStateConstantSpeed();

  // destructor
  ~EgoStateConstantSpeed();

  void onEnter(Ego& ego);

  void onUpdate(Ego& ego);

  void onExit(Ego& ego);
};


#endif //PATH_PLANNING_EGO_STATE_CONSTANT_SPEED_H
