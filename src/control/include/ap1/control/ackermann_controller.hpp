/**
 * Ackermann controller class.
 * This is not a type of icontroller, probably should rename that to be clearer.
 * Created: Nov. 11, 2025
 * Author(s): Aly Ashour
 */

#ifndef AP1_ACKERMANN_CONTROLLER_HPP
#define AP1_ACKERMANN_CONTROLLER_HPP

#include <string>
#include "vectors.hpp"

namespace ap1::control
{
class AckermannController
{
  public:
    struct Config
    {
        double L;             // wheelbase in meters
        double a_max;         // max longitudinal accleration (ms^-2)
        double delta_max;     // max steering angle
        double throttle_gain; // scales accel to throttle [0, 1]
        double brake_gain;    // scales -accel to brake [0, 1]
    };

    explicit AckermannController(const Config &cfg);

    struct Command
    {
        double throttle; // normalized [0, 1]
        double brake; // normalized [0, 1]
        double steering; // in rads
    };

    Command compute_command(const vec3f& acc, const vec3f& vel);

    static Config load_config(const std::string &path);

  private:
    Config cfg_;
};
} // namespace ap1::control

#endif // AP1_ACKERMANN_CONTROLLER
