#include "gcode_reader_node.hpp"
#include "coordinate_conversion.h"
#include "std_msgs/msg/u_int8.hpp"

using std_msgs::msg::UInt8;

GCodeReaderNode::GCodeReaderNode()
    : Node("gcode_reader_node")
{
    declare_parameter<std::string>("gcode_file", "/home/your/path/file.gcode");
    declare_parameter<double>("origin_lat", 0.0);
    declare_parameter<double>("origin_lon", 0.0);

    get_parameter("gcode_file", gcode_file_path_);
    get_parameter("origin_lat", origin_lat_);
    get_parameter("origin_lon", origin_lon_);

    path_pub_ = create_publisher<nav_msgs::msg::Path>("gcode_path", 10);
    hitch_pub_ = create_publisher<UInt8>("/hitch/put", 10);

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
        auto enu = coordinate_conversion::latLonToLocal(
            cmd.latitude, cmd.longitude, origin_lat_, origin_lon_);
        pose.pose.position.x = enu.east;
        pose.pose.position.y = enu.north;
        pose.pose.position.z = cmd.altitude;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    RCLCPP_INFO(this->get_logger(), "Publishing G-code path with %zu waypoints", path_msg.poses.size());
    path_pub_->publish(path_msg);

    executeCommands(commands);
}

void GCodeReaderNode::executeCommands(const std::vector<GCodeCommand>& cmds)
{
    for (const auto& cmd : cmds) {
        switch (cmd.type) {
            case GCodeCommand::Move:
                moveTo(cmd);
                break;
            case GCodeCommand::DropPlow:
                dropPlow();
                break;
            case GCodeCommand::LiftPlow:
                liftPlow();
                break;
        }
    }
}

void GCodeReaderNode::moveTo(const GCodeCommand& cmd)
{
    // Convert to local ENU for future nav2 integration
    auto enu = coordinate_conversion::latLonToLocal(
        cmd.latitude, cmd.longitude, origin_lat_, origin_lon_);

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = "map";
    goal.pose.position.x = enu.east;
    goal.pose.position.y = enu.north;
    goal.pose.position.z = cmd.altitude;
    goal.pose.orientation.w = 1.0;

    // TODO: send NavigateToPose action to Nav2
    (void)goal; // suppress unused warning
}

void GCodeReaderNode::liftPlow()
{
    UInt8 msg;
    msg.data = 2; // up command
    hitch_pub_->publish(msg);
}

void GCodeReaderNode::dropPlow()
{
    UInt8 msg;
    msg.data = 1; // down command
    hitch_pub_->publish(msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GCodeReaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
