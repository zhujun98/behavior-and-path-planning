#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "utilities.hpp"


namespace {

TEST(TestSquareDistance, GeneralTest) {
  ASSERT_EQ(2.0, square_distance(1, 0, 0, 1));
  ASSERT_EQ(17.0, square_distance(-1, 0, 3, 1));
}

TEST(TestDistance, GeneralTest) {
  ASSERT_DOUBLE_EQ(std::sqrt(2.0), distance(1, 0, 0, 1));
  ASSERT_DOUBLE_EQ(std::sqrt(17.0), distance(-1, 0, 3, 1));
}

// TODO: add parseSocketData

}