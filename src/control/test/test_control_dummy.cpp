#include <gtest/gtest.h>
#include <algorithm>

// Stand-in function: replace with #include of real control header later
double clamp_command(double value, double min_val, double max_val) {
  return std::clamp(value, min_val, max_val);
}

TEST(ControlDummyTest, ClampCommandRespectsUpperLimit) {
  EXPECT_DOUBLE_EQ(clamp_command(1.5, -1.0, 1.0), 1.0);
}
