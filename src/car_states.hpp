#ifndef PATH_PLANNING_CAR_STATES_H
#define PATH_PLANNING_CAR_STATES_H

#include <vector>

class Car;

//
// CLR: change to the right lane
// CLL: change to the left lane
// KL: keep lane
// ON: normal state, ready to go to the next state
//
enum class States {CLR, CLL, KL, ON};

class CarState {

protected:
  uint16_t tick_ = 0;

  CarState();

public:

  virtual ~CarState();

  // Check the validity of the transition states one-by-one. If valid.
  // return the next state.
  virtual CarState* getNextState(Car& car) = 0;

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


class CarStateOn : public CarState {

public:

  CarStateOn();

  ~CarStateOn() override;

  CarState* getNextState(Car& car) override;

  void onEnter(Car& car) override;

  void onUpdate(Car& car) override;

  void onExit(Car& car) override;
};


class CarStateKeepLane : public CarState {

public:

  CarStateKeepLane();

  ~CarStateKeepLane() override;

  CarState* getNextState(Car& car) override;

  void onEnter(Car& car) override;

  void onUpdate(Car& car) override;

  void onExit(Car& car) override;
};


class CarStateChangeLaneLeft : public CarState {

public:

  CarStateChangeLaneLeft();

  ~CarStateChangeLaneLeft() override;

  CarState* getNextState(Car& car) override;

  void onEnter(Car& car) override;

  void onUpdate(Car& car) override;

  void onExit(Car& car) override;
};


class CarStateChangeLaneRight : public CarState {
public:

  CarStateChangeLaneRight();

  ~CarStateChangeLaneRight() override;

  CarState* getNextState(Car& car) override;

  void onEnter(Car& car) override;

  void onUpdate(Car& car) override;

  void onExit(Car& car) override;

};


#endif //PATH_PLANNING_CAR_STATES_H