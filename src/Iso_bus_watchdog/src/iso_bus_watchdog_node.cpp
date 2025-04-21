#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include "isobus/hardware_integration/socket_can_interface.hpp"
#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_message.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class IsoBusWatchdog : public rclcpp::Node
{
public:
  IsoBusWatchdog()
  : Node("iso_bus_watchdog")
  {
    // Publishers
    pub_oil_   = create_publisher<std_msgs::msg::Float32>("engine/oil_pressure",    10);
    pub_fuel_  = create_publisher<std_msgs::msg::Float32>("engine/fuel_pressure",   10);
    pub_temp_  = create_publisher<std_msgs::msg::Float32>("engine/coolant_temp",    10);
    pub_level_ = create_publisher<std_msgs::msg::Float32>("engine/fuel_level",      10);

    // Initialize freshness timestamps
    auto now = std::chrono::steady_clock::now();
    last_oil_   = last_fuel_   = last_temp_   = last_level_ = now;

    // --- Setup CAN via AgIsoStack++ ---
    can_driver_ = std::make_shared<isobus::SocketCANInterface>("can0");
    isobus::CANHardwareInterface::set_number_of_can_channels(1);
    isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, can_driver_);

    if (!isobus::CANHardwareInterface::start() || !can_driver_->get_is_valid()) {
      RCLCPP_ERROR(get_logger(), "Failed to start CAN interface on can0"); 
      return;
    }

    // Register PGN callbacks for ANY control function sending these PGNs  [oai_citation_attribution:0‡delgrossoengineering.com](https://delgrossoengineering.com/isobus-docs/classisobus_1_1CANNetworkManager?utm_source=chatgpt.com)
    isobus::CANNetworkManager::CANNetwork.add_any_control_function_parameter_group_number_callback(
      65263, &IsoBusWatchdog::pgn65263_callback, this);
    isobus::CANNetworkManager::CANNetwork.add_any_control_function_parameter_group_number_callback(
      65276, &IsoBusWatchdog::pgn65276_callback, this);

    // Timer to monitor staleness
    timer_ = create_wall_timer(1s, std::bind(&IsoBusWatchdog::check_freshness, this));
  }

  ~IsoBusWatchdog() override
  {
    // Unregister callbacks
    isobus::CANNetworkManager::CANNetwork.remove_any_control_function_parameter_group_number_callback(
      65263, &IsoBusWatchdog::pgn65263_callback, this);
    isobus::CANNetworkManager::CANNetwork.remove_any_control_function_parameter_group_number_callback(
      65276, &IsoBusWatchdog::pgn65276_callback, this);

    isobus::CANHardwareInterface::stop();
  }

private:
  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_oil_, pub_fuel_, pub_temp_, pub_level_;

  // CAN driver handle
  std::shared_ptr<isobus::SocketCANInterface> can_driver_;

  // Freshness check timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Last-received timestamps
  std::chrono::steady_clock::time_point last_oil_, last_fuel_, last_temp_, last_level_;

  // Static callback adapters
  static void pgn65263_callback(const isobus::CANMessage &msg, void *ctx)
  {
    static_cast<IsoBusWatchdog*>(ctx)->process65263(msg);
  }
  static void pgn65276_callback(const isobus::CANMessage &msg, void *ctx)
  {
    static_cast<IsoBusWatchdog*>(ctx)->process65276(msg);
  }

  // Process PGN 65263: SPN 100 (oil), 94 (fuel), 110 (temp)
  void process65263(const isobus::CANMessage &msg)
  {
    auto now = std::chrono::steady_clock::now();
    auto d = msg.get_data();

    float oil  = d[3] * 4.0f;    // SPN 100 (4 kPa/bit)
    float fuel = d[0] * 4.0f;    // SPN 94  (4 kPa/bit)
    float temp = d[5] - 40.0f;   // SPN 110 (1 °C/bit, –40 °C offset)

    std_msgs::msg::Float32 mo; mo.data = oil;  pub_oil_->publish(mo);
    std_msgs::msg::Float32 mf; mf.data = fuel; pub_fuel_->publish(mf);
    std_msgs::msg::Float32 mt; mt.data = temp; pub_temp_->publish(mt);

    last_oil_ = last_fuel_ = last_temp_ = now;
  }

  // Process PGN 65276: SPN 96 (fuel level)
  void process65276(const isobus::CANMessage &msg)
  {
    auto now = std::chrono::steady_clock::now();
    auto d = msg.get_data();

    float level = d[0] * 0.4f; // SPN 96 (0.4 %/bit)
    std_msgs::msg::Float32 ml; ml.data = level; pub_level_->publish(ml);

    last_level_ = now;
  }

  // Log warnings if data stale >2 s
  void check_freshness()
  {
    auto now = std::chrono::steady_clock::now();
    if (now - last_oil_   > 2s) RCLCPP_WARN(get_logger(), "Oil pressure stale (>2 s)");
    if (now - last_fuel_  > 2s) RCLCPP_WARN(get_logger(), "Fuel pressure stale (>2 s)");
    if (now - last_temp_  > 2s) RCLCPP_WARN(get_logger(), "Coolant temp stale (>2 s)");
    if (now - last_level_ > 2s) RCLCPP_WARN(get_logger(), "Fuel level stale (>2 s)");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IsoBusWatchdog>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
