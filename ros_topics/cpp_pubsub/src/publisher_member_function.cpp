#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = vision_msgs::msg::Detection3DArray();
    message.header.stamp = this->now();
    message.header.frame_id = "map";

    // Fake a single dynamic object, mimicking what perception will send
    vision_msgs::msg::Detection3D detection;
    detection.header = message.header;
    detection.id = "0";

    vision_msgs::msg::ObjectHypothesisWithPose result;
    result.hypothesis.class_id = "car";
    result.hypothesis.score = 0.9;
    result.pose.pose.position.x = 1.0 + count_;      // replace with real logic
    result.pose.pose.position.y = 2.0;
    result.pose.pose.position.z = 0.0;
    result.pose.pose.orientation.w = 1.0;
    detection.results.push_back(result);

    detection.bbox.center = result.pose.pose;
    detection.bbox.size.x = 4.0;
    detection.bbox.size.y = 2.0;
    detection.bbox.size.z = 1.5;

    message.detections.push_back(detection);
    ++count_;

    RCLCPP_INFO(this->get_logger(), "Publishing %zu detection(s): x=%.2f y=%.2f z=%.2f",
      message.detections.size(), result.pose.pose.position.x,
      result.pose.pose.position.y, result.pose.pose.position.z);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
