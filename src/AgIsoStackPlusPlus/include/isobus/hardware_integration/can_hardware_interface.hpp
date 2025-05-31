#ifndef ISOBUS_CAN_HARDWARE_INTERFACE_HPP
#define ISOBUS_CAN_HARDWARE_INTERFACE_HPP

#include "isobus/isobus/can_message_frame.hpp"
#include "isobus/utility/event_dispatcher.hpp"
#include "isobus/utility/thread_synchronization.hpp"
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <thread>
#include <vector>

namespace isobus {

class CANHardwarePlugin;

constexpr std::uint32_t PERIODIC_UPDATE_INTERVAL = 100;

template<typename T>
class SimpleQueue {
public:
    explicit SimpleQueue(std::size_t cap = 32) : capacity(cap) {}
    bool push(const T &v) {
        if (is_full()) return false;
        data.push_back(v);
        return true;
    }
    bool peek(T &v) {
        if (data.empty()) return false;
        v = data.front();
        return true;
    }
    void pop() { if (!data.empty()) data.pop_front(); }
    bool is_full() const { return data.size() >= capacity; }
    void clear() { data.clear(); }
private:
    std::deque<T> data;
    std::size_t capacity;
};

class CANHardwareInterface {
public:
    class CANHardware {
    public:
        explicit CANHardware(std::size_t queueCapacity = 32);
        ~CANHardware();
        bool start();
        bool stop();
        bool transmit_can_frame(const CANMessageFrame &frame) const;
        bool receive_can_frame();
#if !defined CAN_STACK_DISABLE_THREADS && !defined ARDUINO
        void start_threads();
        void stop_threads();
        void receive_thread_function();
        std::unique_ptr<std::thread> receiveMessageThread;
        std::atomic_bool receiveThreadRunning{false};
#endif
        std::shared_ptr<CANHardwarePlugin> frameHandler;
        SimpleQueue<CANMessageFrame> messagesToBeTransmittedQueue;
        SimpleQueue<CANMessageFrame> receivedMessagesQueue;
    };

    static bool set_number_of_can_channels(std::uint8_t value, std::size_t queueCapacity = 32);
    static bool assign_can_channel_frame_handler(std::uint8_t channelIndex, std::shared_ptr<CANHardwarePlugin> driver);
    static bool unassign_can_channel_frame_handler(std::uint8_t channelIndex);
    static std::shared_ptr<CANHardwarePlugin> get_assigned_can_channel_frame_handler(std::uint8_t channelIndex);
    static std::uint8_t get_number_of_can_channels();
    static bool start();
    static bool stop();
    static bool is_running();
    static bool transmit_can_frame(const CANMessageFrame &frame);
    static EventDispatcher<const CANMessageFrame &> &get_can_frame_received_event_dispatcher();
    static EventDispatcher<const CANMessageFrame &> &get_can_frame_transmitted_event_dispatcher();
    static EventDispatcher<> &get_periodic_update_event_dispatcher();
    static void set_periodic_update_interval(std::uint32_t value);
    static std::uint32_t get_periodic_update_interval();
    static void update();
#if !defined CAN_STACK_DISABLE_THREADS && !defined ARDUINO
    static void update_thread_function();
    static void start_threads();
    static void stop_threads();
#endif
private:
    static std::vector<std::unique_ptr<CANHardware>> hardwareChannels;
    static Mutex hardwareChannelsMutex;
    static Mutex updateMutex;
#if !defined CAN_STACK_DISABLE_THREADS && !defined ARDUINO
    static std::unique_ptr<std::thread> updateThread;
    static std::condition_variable updateThreadWakeupCondition;
#endif
    static std::uint32_t periodicUpdateInterval;
    static std::uint32_t lastUpdateTimestamp;
    static std::atomic_bool started;
    static EventDispatcher<const CANMessageFrame &> frameReceivedEventDispatcher;
    static EventDispatcher<const CANMessageFrame &> frameTransmittedEventDispatcher;
    static EventDispatcher<> periodicUpdateEventDispatcher;
    static CANHardwareInterface SINGLETON;
};

class CANHardwarePlugin {
public:
    virtual ~CANHardwarePlugin() = default;
    virtual bool get_is_valid() const = 0;
    virtual void open() = 0;
    virtual void close() = 0;
    virtual bool read_frame(CANMessageFrame &frame) = 0;
    virtual bool write_frame(const CANMessageFrame &frame) = 0;
    virtual std::string get_device_name() const { return {}; }
    virtual bool set_name(const std::string &) { return false; }
};

bool send_can_message_frame_to_hardware(const CANMessageFrame &frame);

} // namespace isobus

#endif // ISOBUS_CAN_HARDWARE_INTERFACE_HPP
