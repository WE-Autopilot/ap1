#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "ap1_msgs/msg/float_stamped.hpp"

using ap1_msgs::msg::FloatStamped;
using namespace std::chrono_literals;

class SyntheticActuationFeedbackNode : public rclcpp::Node
{
  public:
    SyntheticActuationFeedbackNode()
        : Node("synthetic_actuation_feedback")
    {
        speed_mps_ = this->declare_parameter<double>("speed_mps", 1.0);
        turn_angle_rad_ =
            this->declare_parameter<double>("turn_angle_rad", 0.0);
        publish_rate_hz_ =
            this->declare_parameter<double>("publish_rate_hz", 20.0);

        if (publish_rate_hz_ <= 0.0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "publish_rate_hz must be > 0; using 20.0 Hz");
            publish_rate_hz_ = 20.0;
        }

        speed_pub_ = this->create_publisher<FloatStamped>(
            "/ap1/actuation/speed",
            10);
        turn_angle_pub_ = this->create_publisher<FloatStamped>(
            "/ap1/actuation/turn_angle",
            10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate_hz_),
            std::bind(
                &SyntheticActuationFeedbackNode::publish_feedback,
                this));

        RCLCPP_INFO(
            this->get_logger(),
            "Synthetic actuation feedback ready. speed: %.2f m/s, "
            "turn_angle: %.2f rad, rate: %.2f Hz",
            speed_mps_,
            turn_angle_rad_,
            publish_rate_hz_);
    }

  private:
    void publish_feedback()
    {
        const auto stamp = this->now();

        FloatStamped speed_msg;
        speed_msg.header.stamp = stamp;
        speed_msg.value = static_cast<float>(speed_mps_);
        speed_pub_->publish(speed_msg);

        FloatStamped turn_msg;
        turn_msg.header.stamp = stamp;
        turn_msg.value = static_cast<float>(turn_angle_rad_);
        turn_angle_pub_->publish(turn_msg);
    }

    double speed_mps_;
    double turn_angle_rad_;
    double publish_rate_hz_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<FloatStamped>::SharedPtr speed_pub_;
    rclcpp::Publisher<FloatStamped>::SharedPtr turn_angle_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SyntheticActuationFeedbackNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
