#include "gcode_reader_node.hpp"

GCodeReaderNode::GCodeReaderNode()
    : Node("gcode_reader_node")
{
    declare_parameter<std::string>("gcode_file", "/home/your/path/file.gcode");
    get_parameter("gcode_file", gcode_file_path_);

    path_pub_ = create_publisher<nav_msgs::msg::Path>("gcode_path", 10);
    loadGCodeAndPublish();
}

void GCodeReaderNode::loadGCodeAndPublish() {
    GCodeParser parser;
    auto commands = parser.parseFile(gcode_file_path_);

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "map";

    for (const auto& cmd : commands) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = cmd.longitude;
        pose.pose.position.y = cmd.latitude;
        pose.pose.position.z = cmd.altitude;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    RCLCPP_INFO(this->get_logger(), "Publishing G-code path with %zu waypoints", path_msg.poses.size());
    path_pub_->publish(path_msg);
}
