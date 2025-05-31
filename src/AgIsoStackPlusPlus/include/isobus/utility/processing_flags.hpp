#ifndef ISOBUS_PROCESSING_FLAGS_HPP
#define ISOBUS_PROCESSING_FLAGS_HPP

#include <cstdint>

namespace isobus {

class ProcessingFlags {
public:
    void set_flag(std::uint32_t flag) { flags |= (1u << flag); }
    void clear_flag(std::uint32_t flag) { flags &= ~(1u << flag); }
    bool check_flag(std::uint32_t flag) const { return (flags & (1u << flag)) != 0; }
    void clear_all() { flags = 0; }
    std::uint32_t get_flags() const { return flags; }
private:
    std::uint32_t flags{0};
};

} // namespace isobus

#endif // ISOBUS_PROCESSING_FLAGS_HPP
