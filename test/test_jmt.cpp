#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "jmt.hpp"

namespace {


TEST(TestJerkMinimizingTrajectory, Coefficients) {
  motion state0_0 = {0, 10, 0};
  motion state1_0 = {10, 10, 0};
  polynomial_coeff p_0 = jerkMinimizingTrajectory(state0_0, state1_0, 1);
  polynomial_coeff gt_0 = {0, 10, 0, 0, 0, 0};
  for (int i = 0; i < 6; ++i) ASSERT_NEAR(gt_0[i], p_0[i], 1e-12);

  motion state0_1 = {0, 10, 0};
  motion state1_1 = {20, 15, 20};
  polynomial_coeff p_1 = jerkMinimizingTrajectory(state0_1, state1_1, 2);
  polynomial_coeff gt_1 = {0, 10, 0, 0, -0.625, 0.3125};
  for (int i = 0; i < 6; ++i) ASSERT_NEAR(gt_1[i], p_1[i], 1e-12);

  motion state0_2 = {5, 10, 2};
  motion state1_2 = {-30, -20, -4};
  polynomial_coeff p_2 = jerkMinimizingTrajectory(state0_2, state1_2, 5);
  polynomial_coeff gt_2 = {5, 10, 1, -3.0, 0.64, -0.0432};
  for (int i = 0; i < 6; ++i) ASSERT_NEAR(gt_2[i], p_2[i], 1e-12);
}


TEST(TestJerkMinimizingTrajectory, Evaluation) {
  motion state0 = {5, 10, 2};
  motion state1 = {-30, -20, -4};
  polynomial_coeff p = jerkMinimizingTrajectory(state0, state1, 5);

  ASSERT_NEAR(5, evalTrajectory(p, 0), 1e-12);
  ASSERT_NEAR(-30, evalTrajectory(p, 5), 1e-12);

  ASSERT_NEAR(10, evalVelocity(p, 0), 1e-12);
  ASSERT_NEAR(-20, evalVelocity(p, 5), 1e-12);

  ASSERT_NEAR(2, evalAcceleration(p, 0), 1e-12);
  ASSERT_NEAR(-4, evalAcceleration(p, 5), 1e-12);
}

}