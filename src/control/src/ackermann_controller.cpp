/**
 * Ackermann Controller implementation bbgl
 * Created: Nov. 11, 2025
 * Author(s): Aly Ashour
 */

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "vectors.hpp"
#include "ap1/control/ackermann_controller.hpp"

#define EPSILON 1e-3

namespace ap1::control
{
// also this should be moved to a csv/config loader
std::unordered_map<std::string, double> load_csv_config(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open config file: " + path);
    }

    std::unordered_map<std::string, double> config;
    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        std::string key, value_str;
        if (std::getline(ss, key, ',') && std::getline(ss, value_str))
        {
            try
            {
                double value = std::stod(value_str);
                config[key] = value;
            }
            catch (const std::exception& e)
            {
                throw std::runtime_error("Invalid vlaue for key " + key + ": " + value_str);
            }
        }
    }

    return config;
}

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

AckermannController::Config AckermannController::load_config(const std::string& path)
{
    Config cfg;

    auto map = load_csv_config(path);

    cfg.L = map["wheelbase"];
    cfg.a_max = map["a_max"];
    cfg.delta_max = map["delta_max"];
    cfg.throttle_gain = map["throttle_gain"];
    cfg.brake_gain = map["brake_gain"];

    return cfg;
}
} // namespace ap1::control
