#ifndef ISOBUS_SOCKET_CAN_INTERFACE_HPP
#define ISOBUS_SOCKET_CAN_INTERFACE_HPP

#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include <linux/can.h>
#include <string>

namespace isobus {

class SocketCANInterface : public CANHardwarePlugin {
public:
    explicit SocketCANInterface(const std::string deviceName);
    ~SocketCANInterface() override;

    bool get_is_valid() const override;
    std::string get_device_name() const override;
    void close() override;
    void open() override;
    bool read_frame(CANMessageFrame &canFrame) override;
    bool write_frame(const CANMessageFrame &canFrame) override;
    bool set_name(const std::string &newName) override;

private:
    sockaddr_can *pCANDevice;
    std::string name;
    int fileDescriptor;
};

} // namespace isobus

#endif // ISOBUS_SOCKET_CAN_INTERFACE_HPP
