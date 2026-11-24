#include <cmath>
#include <cstddef>
#include <deque>
#include <memory>
#include <optional>
#include <stdexcept>

#include "ap1_msgs/msg/float_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

class DeadReckoningNode : public rclcpp::Node
{
  public:
    DeadReckoningNode()
        : Node("dead_reckoning_node"),
          speed_timeout_s_(declare_parameter<double>("speed_timeout_s", 0.5)),
          broadcaster_(*this)
    {
        if (!std::isfinite(speed_timeout_s_) || speed_timeout_s_ <= 0.0)
        {
            throw std::invalid_argument("speed_timeout_s must be finite and positive");
        }

        speed_sub_ = create_subscription<ap1_msgs::msg::FloatStamped>(
            "/ap1/actuation/speed", rclcpp::SensorDataQoS(),
            [this](const ap1_msgs::msg::FloatStamped& msg) { on_speed(msg); });
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/sensing/imu/data", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu& msg) { on_imu(msg); });
    }

  private:
    struct SpeedSample
    {
        rclcpp::Time stamp;
        double speed_mps;
    };

    // Bound the queue even if wheel speed keeps arriving while the IMU is offline.
    static constexpr std::size_t max_speed_samples = 1024;

    void on_speed(const ap1_msgs::msg::FloatStamped& msg)
    {
        const rclcpp::Time stamp(msg.header.stamp);
        if (!std::isfinite(msg.value) ||
            (!speed_samples_.empty() && stamp <= speed_samples_.back().stamp))
        {
            return;
        }

        speed_samples_.push_back({stamp, msg.value});
        if (speed_samples_.size() > max_speed_samples)
        {
            speed_samples_.pop_front();
        }
    }

    double speed_at(const rclcpp::Time& stamp)
    {
        // Keep the newest applicable sample and any newer samples received ahead of the IMU.
        while (speed_samples_.size() > 1 && speed_samples_[1].stamp <= stamp)
        {
            speed_samples_.pop_front();
        }
        if (speed_samples_.empty() || speed_samples_.front().stamp > stamp ||
            (stamp - speed_samples_.front().stamp).seconds() > speed_timeout_s_)
        {
            return 0.0;
        }
        return speed_samples_.front().speed_mps;
    }

    void on_imu(const sensor_msgs::msg::Imu& msg)
    {
        const rclcpp::Time stamp(msg.header.stamp);
        const double yaw_rate = msg.angular_velocity.z;
        if (!std::isfinite(yaw_rate) || (last_imu_stamp_ && stamp <= *last_imu_stamp_))
        {
            return;
        }

        const double speed = speed_at(stamp);
        if (last_imu_stamp_)
        {
            const double dt = (stamp - *last_imu_stamp_).seconds();
            yaw_ += yaw_rate * dt;
            x_ += speed * std::cos(yaw_) * dt;
            y_ += speed * std::sin(yaw_) * dt;
        }
        last_imu_stamp_ = stamp;

        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, yaw_);
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = msg.header.stamp;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.rotation = tf2::toMsg(orientation);
        broadcaster_.sendTransform(transform);
    }

    const double speed_timeout_s_;
    tf2_ros::TransformBroadcaster broadcaster_;
    rclcpp::Subscription<ap1_msgs::msg::FloatStamped>::SharedPtr speed_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::deque<SpeedSample> speed_samples_;
    std::optional<rclcpp::Time> last_imu_stamp_;
    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DeadReckoningNode>());
    rclcpp::shutdown();
    return 0;
}
