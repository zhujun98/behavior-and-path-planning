//
// Created by jun on 7/24/17.
//
#ifndef PATH_PLANNING_EGO_STATE_CONSTANT_SPEED_H
#define PATH_PLANNING_EGO_STATE_CONSTANT_SPEED_H

class Ego;
class EgoState;


class EgoStateConstantSpeed : public EgoState {
private:

  void planPath(Ego& ego);

public:

  // constructor
  EgoStateConstantSpeed();

  // destructor
  ~EgoStateConstantSpeed() override;

  void onEnter(Ego& ego) override;

  void onUpdate(Ego& ego) override;

  void onExit(Ego& ego) override;
};


#endif //PATH_PLANNING_EGO_STATE_CONSTANT_SPEED_H