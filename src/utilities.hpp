#ifndef PATH_PLANNING_UTILITIES_H
#define PATH_PLANNING_UTILITIES_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <assert.h>

//
// For converting back and forth between radians and degrees.
//
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

/**
 * Convert speed from MPH to m/s.
 */
inline double mph2mps(double v) { return v * 4.0 / 9; }

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

/**
 * Get the JSON data from the SocketIO event.
 */
inline std::string parseSocketData(const std::string& s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of('[');
  auto b2 = s.find_first_of('}');
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

#endif //PATH_PLANNING_UTILITIES_H
