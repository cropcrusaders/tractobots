#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <isobus/hardware_integration/socket_can_interface.hpp>
#include <isobus/hardware_integration/can_hardware_interface.hpp>
#include <isobus/isobus/can_network_manager.hpp>
#include <isobus/isobus/can_message.hpp>

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class IsoBusWatchdog : public rclcpp::Node
{
public:
  IsoBusWatchdog()
  : Node("iso_bus_watchdog"), emergency_triggered_(false)
  {
    // --- Declare threshold parameters ---
    oil_min_   = this->declare_parameter<double>("oil_pressure_min",   100.0);
    oil_max_   = this->declare_parameter<double>("oil_pressure_max",   800.0);
    fuel_min_  = this->declare_parameter<double>("fuel_pressure_min",  200.0);
    fuel_max_  = this->declare_parameter<double>("fuel_pressure_max",  600.0);
    temp_min_  = this->declare_parameter<double>("coolant_temp_min",   -10.0);
    temp_max_  = this->declare_parameter<double>("coolant_temp_max",   110.0);
    level_min_ = this->declare_parameter<double>("fuel_level_min",      5.0);
    level_max_ = this->declare_parameter<double>("fuel_level_max",      95.0);

    // Publishers for sensor values
    pub_oil_   = create_publisher<std_msgs::msg::Float32>("engine/oil_pressure",  10);
    pub_fuel_  = create_publisher<std_msgs::msg::Float32>("engine/fuel_pressure", 10);
    pub_temp_  = create_publisher<std_msgs::msg::Float32>("engine/coolant_temp",  10);
    pub_level_ = create_publisher<std_msgs::msg::Float32>("engine/fuel_level",    10);

    // Emergency‑stop publisher
    estop_pub_ = create_publisher<std_msgs::msg::Bool>("emergency_stop", 1);

    // Initialize freshness timestamps
    auto now = std::chrono::steady_clock::now();
    last_oil_   = last_fuel_   = last_temp_   = last_level_ = now;

    // --- Setup CAN via AgIsoStack++ ---
    can_driver_ = std::make_shared<isobus::SocketCANInterface>("can0");
    isobus::CANHardwareInterface::set_number_of_can_channels(1);
    isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, can_driver_);

    if (!isobus::CANHardwareInterface::start() || !can_driver_->get_is_valid()) {
      RCLCPP_ERROR(get_logger(), "Failed to start CAN interface on can0");
      throw std::runtime_error("CAN interface startup failed");
    }

    // Register callbacks for the two PGNs
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
  // --- Member variables ---
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_oil_, pub_fuel_, pub_temp_, pub_level_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    estop_pub_;
  std::shared_ptr<isobus::SocketCANInterface>         can_driver_;
  rclcpp::TimerBase::SharedPtr                        timer_;
  std::chrono::steady_clock::time_point               last_oil_, last_fuel_, last_temp_, last_level_;

  // Thresholds
  double oil_min_, oil_max_, fuel_min_, fuel_max_, temp_min_, temp_max_, level_min_, level_max_;
  bool emergency_triggered_;

  // Helper: trigger emergency only once
  void triggerEmergency(const char *reason)
  {
    if (emergency_triggered_) {
      return;
    }
    emergency_triggered_ = true;
    std_msgs::msg::Bool msg; msg.data = true;
    estop_pub_->publish(msg);
    RCLCPP_FATAL(get_logger(), "EMERGENCY: %s – shutting down node", reason);
    rclcpp::shutdown();
  }

  // Static adapters
  static void pgn65263_callback(const isobus::CANMessage &msg, void *ctx)
  {
    static_cast<IsoBusWatchdog*>(ctx)->process65263(msg);
  }
  static void pgn65276_callback(const isobus::CANMessage &msg, void *ctx)
  {
    static_cast<IsoBusWatchdog*>(ctx)->process65276(msg);
  }

  // Process PGN 65263: oil, fuel pressures & coolant temp
  void process65263(const isobus::CANMessage &msg)
  {
    auto now = std::chrono::steady_clock::now();
    auto d   = msg.get_data();

    float oil  = d[3] * 4.0f;   // SPN 100 (4 kPa/bit)
    float fuel = d[0] * 4.0f;   // SPN 94  (4 kPa/bit)
    float temp = d[5] - 40.0f;  // SPN 110 (1 °C/bit, –40 °C offset)

    // Publish readings
    pub_oil_->publish(std_msgs::msg::Float32{oil});
    pub_fuel_->publish(std_msgs::msg::Float32{fuel});
    pub_temp_->publish(std_msgs::msg::Float32{temp});

    // Update freshness
    last_oil_  = last_fuel_  = last_temp_  = now;

    // Check thresholds
    if (oil  < oil_min_  || oil  > oil_max_)  triggerEmergency("Oil pressure out of range");
    if (fuel < fuel_min_ || fuel > fuel_max_) triggerEmergency("Fuel pressure out of range");
    if (temp < temp_min_ || temp > temp_max_) triggerEmergency("Coolant temp out of range");
  }

  // Process PGN 65276: fuel level
  void process65276(const isobus::CANMessage &msg)
  {
    auto now = std::chrono::steady_clock::now();
    float level = msg.get_data()[0] * 0.4f; // SPN 96 (0.4 %/bit)

    pub_level_->publish(std_msgs::msg::Float32{level});
    last_level_ = now;

    if (level < level_min_ || level > level_max_) {
      triggerEmergency("Fuel level out of range");
    }
  }

  // Warn if data stale (>2s)
  void check_freshness()
  {
    auto now = std::chrono::steady_clock::now();
    if (now - last_oil_   > 2s) RCLCPP_WARN(get_logger(), "Oil pressure stale (>2 s)");
    if (now - last_fuel_  > 2s) RCLCPP_WARN(get_logger(), "Fuel pressure stale (>2 s)");
    if (now - last_temp_  > 2s) RCLCPP_WARN(get_logger(), "Coolant temp stale (>2 s)");
    if (now - last_level_ > 2s) RCLCPP_WARN(get_logger(), "Fuel level stale (>2 s)");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<IsoBusWatchdog>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Node initialization failed: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
