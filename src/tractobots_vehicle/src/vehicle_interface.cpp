#include <rclcpp/rclcpp.hpp>

class VehicleInterface : public rclcpp::Node {
public:
  VehicleInterface() : Node("vehicle_interface") {}
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(VehicleInterface)
