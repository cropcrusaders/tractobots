#ifndef ISOBUS_PROCESSING_FLAGS_HPP
#define ISOBUS_PROCESSING_FLAGS_HPP

#include <cstdint>

namespace isobus
{

using ProcessFlagsCallback = void (*)(std::uint32_t flag, void *parent);

class ProcessingFlags
{
public:
    ProcessingFlags(std::uint32_t numberOfFlags, ProcessFlagsCallback processingCallback, void *parentPointer);
    ~ProcessingFlags();

    void set_flag(std::uint32_t flag);
    void process_all_flags();

private:
    ProcessFlagsCallback callback{nullptr};
    std::uint32_t maxFlag{0};
    std::uint8_t *flagBitfield{nullptr};
    void *parent{nullptr};
};

} // namespace isobus

#endif // ISOBUS_PROCESSING_FLAGS_HPP
