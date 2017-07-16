//
// Created by jun on 7/16/17.
//

#include "car.h"


typedef std::map<int, std::vector<std::vector<int>>> state_pred;


Car::Car() {}

Car::~Car() {}

void Car::advance() {
  traffic_.update_state();
  update_state();
  realize_state();
}

void Car::update_state() {
  ;
}


void Car::realize_state() {
  switch (state_) {
    case KL: realize_keep_lane();
    case LCL: realize_lane_change("L");
    case LCR: realize_lane_change("R");
    case PLCL: realize_prep_lane_change("L");
    case PLCR: realize_prep_lane_change("R");
  }
}

void Car::realize_keep_lane() {
  ;
}

void Car::realize_prep_lane_change(std::string direction) {
  ;
}

void Car::realize_lane_change(std::string direction) {
  ;
}
