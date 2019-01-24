#ifndef BEHAVIOR_AND_PATH_PLANNING_COMMON_H
#define BEHAVIOR_AND_PATH_PLANNING_COMMON_H

#include <vector>

using position = std::pair<double, double>;
using trajectory = std::pair<std::vector<double>, std::vector<double>>;
using dynamics = std::pair<std::vector<double>, std::vector<double>>;
using polynomial_coeff = std::vector<double> ;

#endif //BEHAVIOR_AND_PATH_PLANNING_COMMON_H
