//
// Created by jun on 7/28/17.
//
#ifndef PATH_PLANNING_CAR_TRANSITION_STATES_H
#define PATH_PLANNING_CAR_TRANSITION_STATES_H

class Car;
class CarState;


//
// CL_TO_FT: change lane to follow traffic
// FT_TO_CLL: follow traffic to change lane left
// FT_TO_CLR: follow traffic to change lane right
//
enum TransitionStates { CL_TO_FT, FT_TO_CLL, FT_TO_CLR };


class CarTransitionState {
protected:

  CarTransitionState();

public:

  virtual ~CarTransitionState();

  // check whether the transition is valid.
  virtual bool isValid(Car& car) const = 0;

  // get the next vehicle state
  virtual CarState* getNextState(Car& car) const = 0;
};


class CarTransitionStateFactory {
public:
  CarTransitionStateFactory();

  ~CarTransitionStateFactory();

  // construct a new transition state
  static CarTransitionState* createState(TransitionStates name);
};


class CarTransitionCL2FT : public CarTransitionState {
public:

  CarTransitionCL2FT();

  ~CarTransitionCL2FT() override;

  bool isValid(Car& car) const override;

  CarState* getNextState(Car& car) const override;
};


class CarTransitionFT2CLL : public CarTransitionState {
public:

  CarTransitionFT2CLL();

  ~CarTransitionFT2CLL() override;

  bool isValid(Car& car) const override;

  CarState* getNextState(Car& car) const override;
};


class CarTransitionFT2CLR : public CarTransitionState {
public:

  CarTransitionFT2CLR();

  ~CarTransitionFT2CLR() override;

  bool isValid(Car& car) const override;

  CarState* getNextState(Car& car) const override;
};


#endif //PATH_PLANNING_CAR_TRANSITION_STATES_H