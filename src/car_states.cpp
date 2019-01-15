#include <iostream>

#include "car.hpp"
#include "map.hpp"
#include "car_states.hpp"


/*
 * CarState class
 */

CarState::CarState() = default;

CarState::~CarState() = default;


/*
 * CarStateFactory class
 */

CarStateFactory::CarStateFactory() = default;

CarStateFactory::~CarStateFactory() = default;

CarState* CarStateFactory::createState(States name) {
  switch(name) {
    case States::CLR:
      return new CarStateChangeLaneRight;
    case States::CLL:
      return new CarStateChangeLaneLeft;
    case States::KL:
      return new CarStateKeepLane;
    case States::ON:
      return new CarStateOn;
    default:
      throw std::invalid_argument("Unknown state!");
  }
}

/*
 * CarStateReady class
 */

CarStateOn::CarStateOn() = default;

CarStateOn::~CarStateOn() = default;

CarState* CarStateOn::getNextState(Car &car) {
  return CarStateFactory::createState(States::KL);
}

void CarStateOn::onEnter(Car& car) {
  std::cout << "Enter state: *** ON ***" << std::endl;
}

void CarStateOn::onUpdate(Car &car) {
  car.planPath();
}

void CarStateOn::onExit(Car& car) {
  std::cout << "Exit state: *** ON ***" << std::endl;
}

/*
 * CarStateFollowTraffic class
 */

CarStateKeepLane::CarStateKeepLane() = default;

CarStateKeepLane::~CarStateKeepLane() = default;

CarState* CarStateKeepLane::getNextState(Car &car) {
  return nullptr;
}

void CarStateKeepLane::onEnter(Car& car) {
  std::cout << "Enter state: *** KEEP LANE ***" << std::endl;
}

void CarStateKeepLane::onUpdate(Car &car) {
  ++tick_;
  if (tick_ == 5) {
    car.planPath();
    tick_ = 0;
  }
}

void CarStateKeepLane::onExit(Car& car) {
  std::cout << "Exit state: *** KEEP LANE ***" << std::endl;
}

/*
 * CarStateChangeLaneLeft class
 */

CarStateChangeLaneLeft::CarStateChangeLaneLeft() = default;

CarStateChangeLaneLeft::~CarStateChangeLaneLeft() = default;

CarState* CarStateChangeLaneLeft::getNextState(Car &car) {
  return nullptr;
}

void CarStateChangeLaneLeft::onEnter(Car& car) {
  std::cout << "Enter state: *** CHANGE TO THE LEFT LANE *** from Lane-" << car.getCurrentLaneId()
            << " to Lane-" << car.getTargetLaneId() << std::endl;
}

void CarStateChangeLaneRight::onUpdate(Car& car) {}

void CarStateChangeLaneLeft::onExit(Car& car) {
  std::cout << "Exit state: *** CHANGE TO THE LEFT LANE *** " << std::endl;
}

/*
 * CarStateChangeLaneRight class
 */

CarStateChangeLaneRight::CarStateChangeLaneRight() = default;

CarStateChangeLaneRight::~CarStateChangeLaneRight() = default;

CarState* CarStateChangeLaneRight::getNextState(Car &car) {
  return nullptr;
}

void CarStateChangeLaneRight::onEnter(Car& car) {
  std::cout << "Enter state: *** CHANGE TO THE RIGHT LANE *** from Lane-" << car.getCurrentLaneId()
            << " to Lane-" << car.getTargetLaneId() << std::endl;
}

void CarStateChangeLaneLeft::onUpdate(Car& car) {
  car.planPath();
}

void CarStateChangeLaneRight::onExit(Car& carp) {
  std::cout << "Exit state: *** CHANGE TO THE RIGHT LANE *** " << std::endl;
}