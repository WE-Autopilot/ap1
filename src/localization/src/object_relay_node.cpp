#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

using vision_msgs::msg::Detection3DArray;

// Relays perception objects (ICD #2) to P&C (ICD #10).
class ObjectRelayNode : public rclcpp::Node
{
  public:
    ObjectRelayNode() : Node("object_relay_node")
    {
        this->declare_parameter("topics.input", "/ap1/perception/objects");
        this->declare_parameter("topics.output", "/ap1/localization/objects");

        publisher_ = this->create_publisher<Detection3DArray>(
            this->get_parameter("topics.output").as_string(), 10);
        subscription_ = this->create_subscription<Detection3DArray>(
            this->get_parameter("topics.input").as_string(), 10,
            [this](const Detection3DArray& msg) { on_objects(msg); });
    }

  private:
    void on_objects(const Detection3DArray& msg)
    {
        // Keep empty arrays too: "nothing detected" is still news for P&C.
        latest_objects_ = msg;
        last_rx_time_ = this->now();

        RCLCPP_DEBUG(this->get_logger(), "Got %zu objects", msg.detections.size());
        for (const auto& detection : msg.detections)
        {
            for (const auto& result : detection.results)
            {
                const auto& position = result.pose.pose.position;
                RCLCPP_DEBUG(this->get_logger(), "  id=%s class=%s score=%.2f x=%.2f y=%.2f z=%.2f",
                             detection.id.c_str(), result.hypothesis.class_id.c_str(),
                             result.hypothesis.score, position.x, position.y, position.z);
            }
        }

        // TODO(Ali): transform base_link -> map before publishing /ap1/localization/objects
        publisher_->publish(msg);
    }

    rclcpp::Subscription<Detection3DArray>::SharedPtr subscription_;
    rclcpp::Publisher<Detection3DArray>::SharedPtr publisher_;
    Detection3DArray latest_objects_;
    rclcpp::Time last_rx_time_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectRelayNode>());
    rclcpp::shutdown();
    return 0;
}
