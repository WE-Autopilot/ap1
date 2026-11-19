#include <gtest/gtest.h>

// Stand-in constant: replace with real control config header later
constexpr double kControlLoopPeriodMs = 10.0;

TEST(SmokeTest, ControlLoopPeriodIs100Hz) {
  const double frequency_hz = 1000.0 / kControlLoopPeriodMs;
  EXPECT_DOUBLE_EQ(frequency_hz, 100.0);
}
