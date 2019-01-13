#ifndef PATH_PLANNING_CAR_TRANSITION_STATES_H
#define PATH_PLANNING_CAR_TRANSITION_STATES_H

class Car;
class CarState;


//
// ON_TO_KL: on to keep lane
// CL_TO_KL: change lane to keep lane
// KL_TO_CLL: keep lane to change lane left
// KL_TO_CLR: keep lane to change lane right
//
enum class TransitionStates { ON_TO_KL, CL_TO_KL, KL_TO_CLL, KL_TO_CLR };


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


class CarTransitionON2KL : public CarTransitionState {
public:

  CarTransitionON2KL();

  ~CarTransitionON2KL() override;

  bool isValid(Car& car) const override;

  CarState* getNextState(Car& car) const override;
};


class CarTransitionCL2KL : public CarTransitionState {
public:

  CarTransitionCL2KL();

  ~CarTransitionCL2KL() override;

  bool isValid(Car& car) const override;

  CarState* getNextState(Car& car) const override;
};


class CarTransitionKL2CLL : public CarTransitionState {
public:

  CarTransitionKL2CLL();

  ~CarTransitionKL2CLL() override;

  bool isValid(Car& car) const override;

  CarState* getNextState(Car& car) const override;
};


class CarTransitionKL2CLR : public CarTransitionState {
public:

  CarTransitionKL2CLR();

  ~CarTransitionKL2CLR() override;

  bool isValid(Car& car) const override;

  CarState* getNextState(Car& car) const override;
};


#endif //PATH_PLANNING_CAR_TRANSITION_STATES_H