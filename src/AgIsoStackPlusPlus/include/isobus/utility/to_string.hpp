#ifndef ISOBUS_TO_STRING_HPP
#define ISOBUS_TO_STRING_HPP

#include <string>

namespace isobus {

template<typename T>
std::string to_string(T value) {
    return std::to_string(value);
}

} // namespace isobus

#endif // ISOBUS_TO_STRING_HPP
