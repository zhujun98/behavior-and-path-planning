//
// Created by jun on 7/16/17.
//
#ifndef PATH_PLANNING_UTILITIES_H
#define PATH_PLANNING_UTILITIES_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <assert.h>

#include "ego.h"

//
// For converting back and forth between radians and degrees.
//
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

//
//
//
template <class T>
inline void print1DContainer(T v) {
  for ( auto i : v ) {
    std::cout << i << ", ";
  }
  std::cout << std::endl;
}

//
// Checks if the SocketIO event has JSON data.
//
inline std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

#endif //PATH_PLANNING_UTILITIES_H
