#include <gtest/gtest.h>
#include <vector>

// Stand-in function: replace with #include of real planning header once rewrite lands
std::vector<double> plan_route(double start, double goal) {
  return {start, (start + goal) / 2.0, goal};
}

TEST(PlanningDummyTest, RouteStartsAtStartAndEndsAtGoal) {
  auto route = plan_route(0.0, 10.0);
  ASSERT_FALSE(route.empty());
  EXPECT_DOUBLE_EQ(route.front(), 0.0);
  EXPECT_DOUBLE_EQ(route.back(), 10.0);
}
