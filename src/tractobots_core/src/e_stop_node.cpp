#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

using namespace std::chrono_literals;

class EStopNode : public rclcpp::Node {
public:
  EStopNode() : Node("e_stop_node"), estop_(false) {
    sub_ = create_subscription<std_msgs::msg::Bool>(
        "vehicle/estop", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          estop_ = msg->data;
          pub_->publish(std_msgs::msg::Bool().set__data(!estop_));
        });
    pub_ = create_publisher<std_msgs::msg::Bool>("vehicle/cmd_ok", 10);
    updater_.setHardwareID("tractobots");
    updater_.add("EStop", this, &EStopNode::produce_diagnostics);
    timer_ = create_wall_timer(1s, [this]() { updater_.force_update(); });
  }

private:
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    if (estop_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "E-Stop engaged");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Running");
    }
  }

  bool estop_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater updater_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EStopNode>());
  rclcpp::shutdown();
  return 0;
}
