#ifndef ISOBUS_SYSTEM_TIMING_HPP
#define ISOBUS_SYSTEM_TIMING_HPP

#include <chrono>
#include <cstdint>

namespace isobus {

class SystemTiming {
public:
    static std::uint32_t get_timestamp_ms() {
        auto now = std::chrono::steady_clock::now().time_since_epoch();
        return static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
    }
    static bool time_expired_ms(std::uint32_t start, std::uint32_t interval) {
        std::uint32_t now = get_timestamp_ms();
        return now - start >= interval;
    }
};

} // namespace isobus

#endif // ISOBUS_SYSTEM_TIMING_HPP
