#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gcode_parser.h"
#include "std_msgs/msg/u_int8.hpp"

class GCodeReaderNode : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GCodeReaderNode();

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr hitch_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::TimerBase::SharedPtr startup_timer_;

    void loadGCodeAndPublish();
    void executeNextCommand();
    void moveTo(const GCodeCommand& cmd);
    void liftPlow();
    void dropPlow();

    std::string gcode_file_path_;
    double origin_lat_ = 0.0;
    double origin_lon_ = 0.0;

    std::vector<GCodeCommand> commands_;
    std::size_t command_index_ = 0;
};
