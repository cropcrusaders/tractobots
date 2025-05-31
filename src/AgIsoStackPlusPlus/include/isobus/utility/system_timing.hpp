#ifndef ISOBUS_SYSTEM_TIMING_HPP
#define ISOBUS_SYSTEM_TIMING_HPP

#include <cstdint>

namespace isobus
{
    /**
     * @brief Utility class providing time related helper functions.
     *
     * This interface matches the upstream AgIsoStack library so that the
     * implementations in the src directory can be used without modification.
     */
    class SystemTiming
    {
    public:
        static std::uint32_t get_timestamp_ms();
        static std::uint64_t get_timestamp_us();

        static std::uint32_t get_time_elapsed_ms(std::uint32_t timestamp_ms);
        static std::uint64_t get_time_elapsed_us(std::uint64_t timestamp_us);

        static bool time_expired_ms(std::uint32_t timestamp_ms, std::uint32_t timeout_ms);
        static bool time_expired_us(std::uint64_t timestamp_us, std::uint64_t timeout_us);

    private:
        static std::uint32_t incrementing_difference(std::uint32_t currentValue, std::uint32_t previousValue);
        static std::uint64_t incrementing_difference(std::uint64_t currentValue, std::uint64_t previousValue);

        static std::uint64_t s_timestamp_ms;
        static std::uint64_t s_timestamp_us;
    };

} // namespace isobus

#endif // ISOBUS_SYSTEM_TIMING_HPP
