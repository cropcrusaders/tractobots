#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gcode_parser.h"

class GCodeReaderNode : public rclcpp::Node {
public:
    GCodeReaderNode();

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    void loadGCodeAndPublish();

    std::string gcode_file_path_;
    double origin_lat_ = 0.0;
    double origin_lon_ = 0.0;
};
