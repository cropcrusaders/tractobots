#ifndef ISOBUS_EVENT_DISPATCHER_HPP
#define ISOBUS_EVENT_DISPATCHER_HPP

#include <cstddef>
#include <functional>
#include <unordered_map>
#include <vector>

namespace isobus {

using EventCallbackHandle = std::size_t;

template<typename... Args>
class EventDispatcher
{
public:
    using Callback = std::function<void(Args...)>;

    EventCallbackHandle add_listener(const Callback &cb)
    {
        const EventCallbackHandle handle = nextHandle++;
        listeners.emplace(handle, cb);
        return handle;
    }

    void remove_listener(EventCallbackHandle handle)
    {
        listeners.erase(handle);
    }

    void clear_listeners()
    {
        listeners.clear();
    }

    void invoke(Args... args)
    {
        for (auto &entry : listeners)
        {
            entry.second(args...);
        }
    }

    void call(Args... args) { invoke(args...); }

    std::size_t get_listener_count() const
    {
        return listeners.size();
    }

private:
    std::unordered_map<EventCallbackHandle, Callback> listeners;
    EventCallbackHandle nextHandle{0};
};

} // namespace isobus

#endif // ISOBUS_EVENT_DISPATCHER_HPP
