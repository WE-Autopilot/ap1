/**
 * Ackermann Controller implementation bbgl
 * Created: Nov. 11, 2025
 * Author(s): Aly Ashour
 */

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "vectors.hpp"
#include "ap1/control/ackermann_controller.hpp"

#define EPSILON 1e-3

namespace ap1::control
{
AckermannController::AckermannController(const Config& cfg) : cfg_(cfg)
{
    printf("Created Ackermann Controller with config: a_max=%f, delta_max=%f, L=%f, throttle_gain=%f, brake_gain=%f\n",
           cfg_.a_max, cfg_.delta_max, cfg_.L, cfg_.throttle_gain, cfg_.brake_gain);
}

AckermannController::Command AckermannController::compute_command(const vec3f& acc, const vec3f& vel)
{
    Command cmd{};

    double a_long = std::clamp((double)acc.x, -cfg_.a_max, cfg_.a_max);
    double a_lat = acc.y;

    double speed = vel.length();
    double kappa = (speed > EPSILON) ? a_lat / (speed * speed) : 0.0;
    double delta = std::atan(cfg_.L * kappa);
    cmd.steering = delta = std::clamp(delta, -cfg_.delta_max, cfg_.delta_max);

    if (a_long >= 0.0) {
        cmd.throttle = std::clamp(a_long / cfg_.a_max * cfg_.throttle_gain, 0.0, 1.0);
        cmd.brake = 0.0;
    } else {
        cmd.throttle = 0.0;
        cmd.brake = std::clamp(-a_long / cfg_.a_max * cfg_.brake_gain, 0.0, 1.0);
    }

    return cmd;
}

} // namespace ap1::control
