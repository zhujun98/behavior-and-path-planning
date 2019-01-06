//
// Created by jun on 7/24/17.
//
#ifndef PATH_PLANNING_CAR_STATES_H
#define PATH_PLANNING_CAR_STATES_H

#include <vector>

class Car;
class CarTransitionState;

//
// CLR: change to the right lane
// CLL: change to the left lane
// FT: follow traffic (keep lane)
// ON: normal state, ready to go to the next state
//
enum States {CLR, CLL, FT, ON};

class CarState {

protected:
  CarState();

  // Storing the transition states which is belong to this state.
  // The order of the states matter!
  std::vector<CarTransitionState *> transition_states_;

  double timer_;  // track how long the state lasts

public:

  virtual ~CarState();

  // Check the validity of the transition states one-by-one. If valid.
  // Switch to the next state.
  CarState* checkTransition(Car& car);

  // action when entering a state
  virtual void onEnter(Car& car) = 0;

  // action when updating a state
  virtual void onUpdate(Car& car) = 0;

  // action when exiting a state
  virtual void onExit(Car& car) = 0;

};


class CarStateFactory {

public:

  CarStateFactory();

  ~CarStateFactory();

  // construct a new state
  static CarState* createState(States name);
};


class CarStateFollowTraffic : public CarState {

public:

  CarStateFollowTraffic();

  ~CarStateFollowTraffic() override;

  void onEnter(Car& car) override;

  void onUpdate(Car& car) override;

  void onExit(Car& car) override;
};


class CarStateOn : public CarState {

public:

  CarStateOn();

  ~CarStateOn() override;

  void onEnter(Car& car) override;

  void onUpdate(Car& car) override;

  void onExit(Car& car) override;
};


class CarStateChangeLaneLeft : public CarState {

public:

  CarStateChangeLaneLeft();

  ~CarStateChangeLaneLeft() override;

  void onEnter(Car& car) override;

  void onUpdate(Car& car) override;

  void onExit(Car& car) override;
};


class CarStateChangeLaneRight : public CarState {
public:

  CarStateChangeLaneRight();

  ~CarStateChangeLaneRight() override;

  void onEnter(Car& car) override;

  void onUpdate(Car& car) override;

  void onExit(Car& car) override;

};


#endif //PATH_PLANNING_CAR_STATES_H