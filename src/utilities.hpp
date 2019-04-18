#ifndef PATH_PLANNING_UTILITIES_H
#define PATH_PLANNING_UTILITIES_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <assert.h>


/**
 * Compute the squared distance between two points.
 */
inline double square_distance(double px0, double py0, double px1, double py1) {
  double dx = px1 - px0;
  double dy = py1 - py0;

  return dx*dx + dy*dy;
}

/**
 * Compute the distance between two points.
 */
inline double distance(double px0, double py0, double px1, double py1) {
  return std::sqrt(square_distance(px0, py0, px1, py1));
}

#endif //PATH_PLANNING_UTILITIES_H
