#ifndef ISOBUS_DATA_SPAN_HPP
#define ISOBUS_DATA_SPAN_HPP

#include <cstddef>

namespace isobus {

template<typename T>
class DataSpan {
public:
    DataSpan() : ptr(nullptr), len(0) {}
    DataSpan(T *p, std::size_t l) : ptr(p), len(l) {}
    T *data() const { return ptr; }
    std::size_t size() const { return len; }
    bool empty() const { return len == 0; }
    T &operator[](std::size_t idx) const { return ptr[idx]; }
    T *begin() const { return ptr; }
    T *end() const { return ptr + len; }
protected:
    T *ptr;
    std::size_t len;
};

} // namespace isobus

#endif // ISOBUS_DATA_SPAN_HPP
