//
// Created by jun on 7/22/17.
//

#include "behavior_planner.h"


BehaviorPlanner::BehaviorPlanner(Ego& car) {
  car_ = &car;
}

BehaviorPlanner::~BehaviorPlanner() {}

void BehaviorPlanner::plan() {
  car_->setBehavior(KL);
}
