#include "rclcpp/rclcpp.hpp"
#include "ap1/planning/planner_node.hpp"

const char* LOGGER_NAME = "main";

// should ideally provide rate_hz parsed from config file
// leaving this for when launch files are created
const float RATE_HZ = 60;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // get planning node config path from args
    std::string config_path = "";
    if (argc > 1)
    {
        config_path = argv[1];
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Usage: planner_node <config.csv>");
        return 1;
    }
    auto node = std::make_shared<ap1::planning::PlannerNode>(RATE_HZ, argv[1]);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
