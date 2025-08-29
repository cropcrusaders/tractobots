#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class AgOpenGPSBridge : public rclcpp::Node {
public:
  AgOpenGPSBridge() : Node("agopengps_bridge") {}
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(AgOpenGPSBridge)
