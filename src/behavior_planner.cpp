//
// Created by jun on 7/22/17.
//


#include "behavior_planner.h"


BehaviorPlanner::BehaviorPlanner(Ego& car) {
  car_ = &car;
}

BehaviorPlanner::~BehaviorPlanner() {}

void BehaviorPlanner::plan() {

  VehicleBehavior current_behavior = car_->getBehavior();

  if ( current_behavior == LCING ) {
    if ( car_->getLaneID() == car_->getTargetLaneID() ) {
      std::cout << "Lane change finished!" << std::endl;
      car_->setBehavior(KL);
      return;
    }
  }

  if ( current_behavior == KL && std::rand()%100 < 2 ) {
    car_->setBehavior(LC);

    if ( car_->getLaneID() < 3 ) {
      car_->setTargetLaneID(car_->getLaneID() + 1);
      std::cout << "Start changing to the right lane!" << std::endl;
    } else if (car_->getLaneID() == 3 ) {
      car_->setTargetLaneID(car_->getLaneID() - 1);
      std::cout << "Start changing to the left lane!" << std::endl;
    }
  }
}
