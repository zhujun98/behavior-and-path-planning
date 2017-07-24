//
// Created by jun on 7/22/17.
//


#include "behavior_planner.h"


BehaviorPlanner::BehaviorPlanner(Ego& ego) {
  ego_ = &ego;
}

BehaviorPlanner::~BehaviorPlanner() {}

void BehaviorPlanner::plan() {

  VehicleBehavior current_behavior = ego_->getBehavior();

  if ( current_behavior == LC ) {
    if ( ego_->getLaneID() == ego_->getTargetLaneID() ) {
      std::cout << "Lane change finished!" << std::endl;
      ego_->setBehavior(KL);
      ego_->setLaneChangeTimer(0);
      return;
    } else if ( ego_->getLaneChangeTimer() <= 0 ) {
      std::cout << "Lane change time up!";
      ego_->setBehavior(KL);
      ego_->setLaneChangeTimer(0);
      return;
    }
  }

  if ( current_behavior == KL && std::rand()%100 < 2 ) {
    ego_->setBehavior(PLC);

    if ( ego_->getLaneID() < 3 ) {
      ego_->setTargetLaneID(ego_->getLaneID() + 1);
      std::cout << "Start changing to the right lane!" << std::endl;
    } else if (ego_->getLaneID() == 3 ) {
      ego_->setTargetLaneID(ego_->getLaneID() - 1);
      std::cout << "Start changing to the left lane!" << std::endl;
    }

    ego_->setLaneChangeTimer(3);
  }
}
