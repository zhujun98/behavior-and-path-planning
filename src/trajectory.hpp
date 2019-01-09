//
// Created by jun on 12/10/18.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <exception>

#include "utilities.hpp"
#include "spline/spline.h"

using position = std::pair<double, double>;
using trajectory = std::vector<double>;


/**
 * Calculate the index of the closest waypoint.
 */
std::size_t closestWaypoint(double x, double y, const trajectory& traj_x, const trajectory& traj_y);

/**
 * Get the index of the next waypoint.
 */
std::size_t nextWaypoint(double x, double y, double yaw, const trajectory& traj_x, const trajectory& traj_y);


/**
 * Convert the Cartesian coordinate in a trajectory to the Frenet coordinate.
 */
position cartesianToFrenet(double x, double y, double yaw, double max_s,
                           const trajectory& traj_x, const trajectory& traj_y);

/**
 * Convert the Frenet coordinate in a trajectory to the Cartesian coordinate.
 */
position frenetToCartesian(double s, double d, const trajectory& traj_s, double max_s,
                           const trajectory& traj_x, const trajectory& traj_y);

#endif //PATH_PLANNING_TRAJECTORY_H
