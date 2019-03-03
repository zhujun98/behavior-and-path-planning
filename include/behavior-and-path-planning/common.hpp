#ifndef BEHAVIOR_AND_PATH_PLANNING_COMMON_H
#define BEHAVIOR_AND_PATH_PLANNING_COMMON_H

#include <vector>
#include <array>
#include <map>

using position = std::pair<double, double>;
using trajectory = std::pair<std::vector<double>, std::vector<double>>;
using motion = std::array<double, 3>;
using dynamics = std::pair<motion, motion>;
using cars_on_road = std::map<uint16_t, dynamics>;
using polynomial_coeff = std::vector<double> ;

#endif //BEHAVIOR_AND_PATH_PLANNING_COMMON_H
