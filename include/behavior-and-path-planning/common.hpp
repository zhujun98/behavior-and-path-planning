#ifndef BEHAVIOR_AND_PATH_PLANNING_COMMON_H
#define BEHAVIOR_AND_PATH_PLANNING_COMMON_H

#include <vector>
#include <array>
#include <map>


constexpr double pi() { return M_PI; }

constexpr double kMaxSpeedInMPH = 47.5; // in mph
constexpr double kMaxAcc = 9.5; // in m/s^2
constexpr double kMaxJerk= 9.5; // in m/s^3
constexpr double kTimeStep = 0.02; // in second

constexpr uint16_t kPathPlanningInterval = 5; // the frequency of planning a new path
constexpr uint16_t kChangeLaneMaxAttempts = 3; // max number of attempts in searching a path for lane change

// When the speed of car is higher than this value, the car will exit the StartUpState
constexpr double kMaxStartUpSpeed = 10; // in m/s

using position = std::pair<double, double>;
using trajectory = std::pair<std::vector<double>, std::vector<double>>;
using motion = std::array<double, 3>;
using dynamics = std::pair<motion, motion>;
using cars_on_road = std::map<uint16_t, dynamics>;
using polynomial_coeff = std::vector<double> ;

#endif //BEHAVIOR_AND_PATH_PLANNING_COMMON_H
