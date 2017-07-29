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

//
// Estimate the state vector (s, d) at the end of the current path
//
inline std::pair<std::vector<double>, std::vector<double>> getState0(const Ego& ego) {
  double ps0, vs0, as0;
  double pd0, vd0, ad0;

  if ( ego.getPathS()->empty() || ego.getPathS()->size() < 3 ) {
    ps0 = ego.getPs();
    pd0 = ego.getPd();
    vs0 = ego.getVs();
    vd0 = ego.getVd();
    as0 = ego.getAs();
    ad0 = ego.getAd();

  } else {
    auto it_s = ego.getPathS()->rbegin();
    auto it_d = ego.getPathD()->rbegin();
    ps0 = *it_s;
    pd0 = *it_d;
    std::advance(it_s, 1);
    std::advance(it_d, 1);
    double ps0_1 = *it_s;
    double pd0_1 = *it_d;
    std::advance(it_s, 1);
    std::advance(it_d, 1);
    double ps0_2 = *it_s;
    double pd0_2 = *it_d;
    vs0 = (ps0 - ps0_1) / ego.getTimeStep();
    vd0 = (pd0 - pd0_1) / ego.getTimeStep();
    double vs0_1 = (ps0_1 - ps0_2) / ego.getTimeStep();
    double vd0_1 = (pd0_1 - pd0_2) / ego.getTimeStep();
    as0 = (vs0 - vs0_1) / ego.getTimeStep();
    ad0 = (vd0 - vd0_1) / ego.getTimeStep();
  }

  std::vector<double> state0_s {ps0, vs0, as0};
  std::vector<double> state0_d {pd0, vd0, ad0};

  return std::make_pair(state0_s, state0_d);
}

#endif //PATH_PLANNING_UTILITIES_H
