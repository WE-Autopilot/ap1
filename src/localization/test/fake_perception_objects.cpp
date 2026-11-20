#include <chrono>
#include <memory>

#include "ap1_msgs/msg/object_class.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

using namespace std::chrono_literals;
using vision_msgs::msg::Detection3DArray;

// Stands in for perception: publishes one fake pedestrian on /ap1/perception/objects.
class FakePerceptionObjects : public rclcpp::Node
{
  public:
    FakePerceptionObjects() : Node("fake_perception_objects"), count_(0)
    {
        this->declare_parameter("topics.output", "/ap1/perception/objects");

        publisher_ = this->create_publisher<Detection3DArray>(
            this->get_parameter("topics.output").as_string(), 10);
        timer_ = this->create_wall_timer(100ms, [this]() { timer_callback(); });
    }

  private:
    void timer_callback()
    {
        auto message = Detection3DArray();
        message.header.stamp = this->now();
        message.header.frame_id = "base_link";

        vision_msgs::msg::Detection3D detection;
        detection.header = message.header;
        detection.id = "1"; // stable track id

        vision_msgs::msg::ObjectHypothesisWithPose result;
        result.hypothesis.class_id = ap1_msgs::msg::ObjectClass::PEDESTRIAN;
        result.hypothesis.score = 0.9;
        // Walks from 5 m to 10 m ahead, then resets.
        result.pose.pose.position.x = 5.0 + 0.1 * (count_ % 50);
        result.pose.pose.position.y = 2.0;
        result.pose.pose.position.z = 0.0;
        result.pose.pose.orientation.w = 1.0;
        detection.results.push_back(result);

        detection.bbox.center = result.pose.pose;
        detection.bbox.size.x = 0.5;
        detection.bbox.size.y = 0.5;
        detection.bbox.size.z = 1.8;

        message.detections.push_back(detection);
        ++count_;

        RCLCPP_DEBUG(this->get_logger(), "Publishing %zu detection(s): x=%.2f y=%.2f z=%.2f",
                     message.detections.size(), result.pose.pose.position.x,
                     result.pose.pose.position.y, result.pose.pose.position.z);
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Detection3DArray>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakePerceptionObjects>());
    rclcpp::shutdown();
    return 0;
}
