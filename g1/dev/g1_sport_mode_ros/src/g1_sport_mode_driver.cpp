#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"

class G1SportModeDriver : public rclcpp::Node
{
public:
  G1SportModeDriver()
  : Node("g1_sport_mode_driver"), last_command_time_(this->now())
  {
    this->declare_parameter("command_timeout_sec", 0.5);
    this->declare_parameter("max_linear_speed", 0.5);
    this->declare_parameter("max_angular_speed", 1.0);
    this->declare_parameter("deadband", 0.02);

    command_timeout_sec_ = this->get_parameter("command_timeout_sec").as_double();
    max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
    deadband_ = this->get_parameter("deadband").as_double();
    validate_parameters();

    request_publisher_ = this->create_publisher<unitree_api::msg::Request>(
      "/api/sport/request", 10);
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&G1SportModeDriver::cmd_vel_callback, this, std::placeholders::_1));
    watchdog_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(std::min(0.1, command_timeout_sec_ / 2.0))),
      std::bind(&G1SportModeDriver::watchdog_callback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "G1 sport driver started (timeout %.2fs, linear limit %.2f, angular limit %.2f)",
      command_timeout_sec_, max_linear_speed_, max_angular_speed_);
  }

  ~G1SportModeDriver() override
  {
    if (request_publisher_) {
      publish_command(0.0, 0.0, 0.0);
    }
  }

private:
  void validate_parameters() const
  {
    if (!std::isfinite(command_timeout_sec_) || command_timeout_sec_ <= 0.0 ||
      !std::isfinite(max_linear_speed_) || max_linear_speed_ <= 0.0 ||
      !std::isfinite(max_angular_speed_) || max_angular_speed_ <= 0.0 ||
      !std::isfinite(deadband_) || deadband_ < 0.0) {
      throw std::invalid_argument(
        "command_timeout_sec/max speeds must be finite and positive; deadband must be non-negative");
    }
  }

  double sanitize(double value, double limit) const
  {
    if (!std::isfinite(value) || std::abs(value) <= deadband_) {
      return 0.0;
    }
    return std::max(-limit, std::min(limit, value));
  }

  void publish_command(double vx, double vy, double vyaw)
  {
    auto request = std::make_shared<unitree_api::msg::Request>();
    request->header.identity.api_id = 1004;
    request->parameter = "{\n    \"vx\": " + std::to_string(vx) +
      ",\n    \"vy\": " + std::to_string(vy) +
      ",\n    \"vyaw\": " + std::to_string(vyaw) + "\n}";
    request_publisher_->publish(*request);
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double vx = sanitize(msg->linear.x, max_linear_speed_);
    const double vy = sanitize(msg->linear.y, max_linear_speed_);
    const double vyaw = sanitize(msg->angular.z, max_angular_speed_);

    publish_command(vx, vy, vyaw);
    last_command_time_ = this->now();
    motion_active_ = vx != 0.0 || vy != 0.0 || vyaw != 0.0;
  }

  void watchdog_callback()
  {
    if (!motion_active_ ||
      (this->now() - last_command_time_).seconds() <= command_timeout_sec_) {
      return;
    }
    publish_command(0.0, 0.0, 0.0);
    motion_active_ = false;
    RCLCPP_WARN(this->get_logger(), "/cmd_vel timed out; published a stop command");
  }

  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_command_time_;
  double command_timeout_sec_{};
  double max_linear_speed_{};
  double max_angular_speed_{};
  double deadband_{};
  bool motion_active_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<G1SportModeDriver>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("g1_sport_mode_driver"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
