#ifndef ISOBUS_TO_STRING_HPP
#define ISOBUS_TO_STRING_HPP

#include <string>

namespace isobus {

template<typename T>
std::string to_string(T value)
{
    return std::to_string(value);
}

inline std::string to_string(const std::string &value)
{
    return value;
}

inline std::string to_string(const char *value)
{
    return std::string(value);
}

} // namespace isobus

#endif // ISOBUS_TO_STRING_HPP
