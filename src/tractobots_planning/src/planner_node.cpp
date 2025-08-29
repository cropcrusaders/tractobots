#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode() : Node("planner_node") {
    rows_pub_ = create_publisher<nav_msgs::msg::Path>("planning/rows", 10);
    turns_pub_ = create_publisher<nav_msgs::msg::Path>("planning/turns", 10);
  }
private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rows_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr turns_pub_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PlannerNode)
