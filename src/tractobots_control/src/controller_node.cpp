#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode() : Node("controller_node") {
    state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "control/state", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr) {});
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("control/cmd", 10);
  }
private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ControllerNode)
