#include <rclcpp/logger.hpp>
#include <string>

#include "ap1/control/control_node.hpp"

const char* LOGGER_NAME = "main";

int main(int argc, char** argv)
{
    // init ros & ros logger
    rclcpp::init(argc, argv);

    // get control node config path from args
    std::string config_path = "";
    if (argc > 1)
    {
        config_path = argv[1];
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Usage: control_node <config.csv>");
        return 1;
    }

    auto node = std::make_shared<ap1::control::ControlNode>(config_path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
