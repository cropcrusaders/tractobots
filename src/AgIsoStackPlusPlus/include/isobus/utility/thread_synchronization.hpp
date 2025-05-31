#ifndef ISOBUS_THREAD_SYNCHRONIZATION_HPP
#define ISOBUS_THREAD_SYNCHRONIZATION_HPP

#include <mutex>

namespace isobus {

using Mutex = std::mutex;

} // namespace isobus

#define LOCK_GUARD(type, mutex) std::lock_guard<type> lock_guard_##mutex(mutex)

#endif // ISOBUS_THREAD_SYNCHRONIZATION_HPP
