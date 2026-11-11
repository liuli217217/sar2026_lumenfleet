#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "unitree_api/msg/request.hpp"

// Single-threaded executor: callbacks and timer share state serially.
class Go2CmdVelBridge : public rclcpp::Node
{
  using Clock = std::chrono::steady_clock;
public:
  Go2CmdVelBridge() : Node("go2_cmd_vel_bridge")
  {
    enabled_ = parameter<bool>("enabled", false);
    const auto input = parameter<std::string>("cmd_vel_topic", "/lumen/teleop/cmd_vel");
    const auto output = parameter<std::string>("request_topic", "/lumen/test/sport_request");
    const double rate = parameter<double>("publish_rate_hz", 50.0);
    timeout_ = parameter<double>("cmd_timeout_s", 0.25);
    max_vx_ = parameter<double>("max_vx", 0.15);
    max_vy_ = parameter<double>("max_vy", 0.10);
    max_wz_ = parameter<double>("max_wz", 0.30);
    for (double value : {rate, timeout_, max_vx_, max_vy_, max_wz_}) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("Rates, timeout and limits must be finite and positive");
      }
    }
    if (rate > 200.0 || rate < 1.0 || timeout_ > 1.0) {
      throw std::invalid_argument("Rate must be 1..200 Hz and timeout <= 1 second");
    }
    if (get_parameter("use_sim_time").as_bool()) {
      throw std::invalid_argument("Motion bridge requires use_sim_time=false");
    }
    if (!enabled_) {
      RCLCPP_INFO(get_logger(), "DISABLED: no command subscription or request publisher");
      return;
    }
    pub_ = create_publisher<unitree_api::msg::Request>(output, rclcpp::QoS(1));
    sub_ = create_subscription<geometry_msgs::msg::Twist>(input, rclcpp::QoS(1),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        last_ = Clock::now();
        have_command_ = true;
        // Reject nonfinite values anywhere, including unused axes.
        valid_ = std::isfinite(msg->linear.x) && std::isfinite(msg->linear.y) &&
          std::isfinite(msg->linear.z) && std::isfinite(msg->angular.x) &&
          std::isfinite(msg->angular.y) && std::isfinite(msg->angular.z);
        command_ = *msg;
        if (!valid_) RCLCPP_WARN(get_logger(), "Invalid velocity: requesting stop");
      });
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate)), [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "ENABLED: input=%s output=%s; no requests until first input",
      input.c_str(), output.c_str());
  }
private:
  template<typename T> T parameter(const std::string &name, const T &value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }
  void tick() {
    if (!have_command_) return;
    const double age = std::chrono::duration<double>(Clock::now() - last_).count();
    unitree_api::msg::Request request;
    if (!valid_ || age >= timeout_ ||
        (command_.linear.x == 0 && command_.linear.y == 0 && command_.angular.z == 0)) {
      // Repeat stop while alive; this is not an acknowledgement or crash watchdog.
      request.header.identity.api_id = 1003;
    } else {
      request.header.identity.api_id = 1008;
      std::ostringstream json;
      json.imbue(std::locale::classic());
      json << std::fixed << std::setprecision(6)
        << "{\"x\":" << std::clamp(command_.linear.x, -max_vx_, max_vx_)
        << ",\"y\":" << std::clamp(command_.linear.y, -max_vy_, max_vy_)
        << ",\"z\":" << std::clamp(command_.angular.z, -max_wz_, max_wz_) << "}";
      request.parameter = json.str();
    }
    pub_->publish(request);
  }
  bool enabled_{false}, have_command_{false}, valid_{false};
  double timeout_, max_vx_, max_vy_, max_wz_;
  Clock::time_point last_;
  geometry_msgs::msg::Twist command_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Go2CmdVelBridge>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("go2_cmd_vel_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
