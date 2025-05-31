#ifndef ARDUINO_STUB_H
#define ARDUINO_STUB_H

#include <cstdint>
#include <cstddef>
#include <chrono>
#include <thread>

inline void delay(unsigned long ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

inline unsigned long millis()
{
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return static_cast<unsigned long>(std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count());
}

class HardwareSerial
{
public:
    template<typename T>
    void print(const T &) {}

    template<typename T>
    void println(const T &) {}
};

static HardwareSerial Serial;

#endif // ARDUINO_STUB_H
