#include "ap1/planning/planner_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ap1::planning::PlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
