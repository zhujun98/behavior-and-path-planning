#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <exception>

#include "utilities.hpp"
#include "spline/spline.h"

using position = std::pair<double, double>;


/**
 * Calculate the index of the closest waypoint.
 */
std::size_t closestWaypoint(double x, double y, 
                            const std::vector<double>& traj_x, const std::vector<double>& traj_y);

/**
 * Get the index of the next waypoint.
 */
std::size_t nextWaypoint(double x, double y, double yaw, 
                         const std::vector<double>& traj_x, const std::vector<double>& traj_y);


/**
 * Convert the Cartesian coordinate in a trajectory to the Frenet coordinate.
 */
position cartesianToFrenet(double x, double y, double yaw, double max_s,
                           const std::vector<double>& traj_x, const std::vector<double>& traj_y);

/**
 * Convert the Frenet coordinate in a trajectory to the Cartesian coordinate.
 */
position frenetToCartesian(double s, double d, const std::vector<double>& traj_s, double max_s,
                           const std::vector<double>& traj_x, const std::vector<double>& traj_y);

#endif //PATH_PLANNING_TRAJECTORY_H
