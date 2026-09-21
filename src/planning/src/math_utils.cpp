#include "ap1/planning/math_utils.hpp"
#include <cmath>

namespace ap1::math {

float distance(const vec2f& next, const vec2f& target)
{
    // Formula: sqrt((b.x - a.x)^2 + (b.y - a.y)^2)
    float distance = std::sqrt(
                std::pow((target.x - next.x), 2) +
                std::pow((target.y - next.y), 2)
                );
    return distance;
}

float magnitude(float x, float y) {
    return std::sqrt(x*x + y*y);
}

float magnitude(const vec2f& v) {
    return std::sqrt(v.x*v.x + v.y*v.y);
}

} // namespace ap1::math
