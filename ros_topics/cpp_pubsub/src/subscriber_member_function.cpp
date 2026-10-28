#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const vision_msgs::msg::Detection3DArray & msg) const
  {
    for (const auto & detection : msg.detections) {
      // Pose is the important part for now
      for (const auto & result : detection.results) {
        const auto & position = result.pose.pose.position;
        RCLCPP_INFO(this->get_logger(),
          "I heard: id=%s class=%s score=%.2f x=%.2f y=%.2f z=%.2f",
          detection.id.c_str(), result.hypothesis.class_id.c_str(),
          result.hypothesis.score, position.x, position.y, position.z);
      }
    }
  }
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
