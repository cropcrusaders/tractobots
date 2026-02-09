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
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    loadGCodeAndPublish();
}

void GCodeReaderNode::loadGCodeAndPublish() {
    GCodeParser parser;
    commands_ = parser.parseFile(gcode_file_path_);

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "map";

    for (const auto& cmd : commands_) {
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

    // Defer command execution until the node is spinning and Nav2 is available
    command_index_ = 0;
    startup_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
            if (!nav_client_->action_server_is_ready()) {
                RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server...");
                return;
            }
            startup_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Nav2 action server available, executing %zu commands",
                commands_.size());
            executeNextCommand();
        });
}

void GCodeReaderNode::executeNextCommand()
{
    if (command_index_ >= commands_.size()) {
        RCLCPP_INFO(this->get_logger(), "All G-code commands executed");
        return;
    }

    const auto& cmd = commands_[command_index_];
    switch (cmd.type) {
        case GCodeCommand::Move:
            moveTo(cmd);
            // moveTo advances command_index_ and calls executeNextCommand via result callback
            return;
        case GCodeCommand::DropPlow:
            dropPlow();
            break;
        case GCodeCommand::LiftPlow:
            liftPlow();
            break;
    }
    command_index_++;
    executeNextCommand();
}

void GCodeReaderNode::moveTo(const GCodeCommand& cmd)
{
    auto enu = coordinate_conversion::latLonToLocal(
        cmd.latitude, cmd.longitude, origin_lat_, origin_lon_);

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.stamp = now();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = enu.east;
    goal_msg.pose.pose.position.y = enu.north;
    goal_msg.pose.pose.position.z = cmd.altitude;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Sending NavigateToPose goal [%zu/%zu]: x=%.2f, y=%.2f",
        command_index_ + 1, commands_.size(), enu.east, enu.north);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this](const GoalHandleNavigate::SharedPtr & goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal was rejected");
                command_index_++;
                executeNextCommand();
            }
        };

    send_goal_options.result_callback =
        [this](const GoalHandleNavigate::WrappedResult & result) {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "NavigateToPose goal succeeded");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "NavigateToPose goal canceled");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "NavigateToPose unknown result code");
                    break;
            }
            command_index_++;
            executeNextCommand();
        };

    nav_client_->async_send_goal(goal_msg, send_goal_options);
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
