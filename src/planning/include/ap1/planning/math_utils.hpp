#ifndef AP1_PLANNING_MATH_UTILS_HPP
#define AP1_PLANNING_MATH_UTILS_HPP

namespace ap1::math {
/**
 * @brief 2D vector with floating point coordinates
 *
 * Used to represent positions, waypoints, and directions in 2D space.
 * Lighter weight than geometry_msgs::Point since we don't need the z coordinate
 * for ground vehicle navigation.
 */
struct vec2f {
    float x;  // meters (forward is positive)
    float y;  // meters (left is positive)

    vec2f() : x(0.0f), y(0.0f) {}
    vec2f(float x, float y) : x(x), y(y) {}
};

/**
 * @brief Calculate the Euclidean distance between two 2D points
 *
 * @param a First point
 * @param b Second point
 * @return float Distance in meters
 */
float distance(const vec2f& next, const vec2f& target);

/**
 * @brief Calculate the magnitude of a 2d vector
 */
float magnitude(const vec2f& v);
float magnitude(float x, float y);
} // ap1::math

#endif // AP1_PLANNING_MATH_UTILS_HPP
