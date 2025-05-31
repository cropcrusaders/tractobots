#ifndef ISOBUS_EVENT_DISPATCHER_HPP
#define ISOBUS_EVENT_DISPATCHER_HPP

#include <functional>
#include <vector>

namespace isobus {

template<typename... Args>
class EventDispatcher {
public:
    using Callback = std::function<void(Args...)>;
    void add_listener(const Callback &cb) { listeners.push_back(cb); }
    void clear_listeners() { listeners.clear(); }
    void invoke(Args... args) {
        for (auto &cb : listeners) {
            cb(args...);
        }
    }
    void call(Args... args) { invoke(args...); }
private:
    std::vector<Callback> listeners;
};

} // namespace isobus

#endif // ISOBUS_EVENT_DISPATCHER_HPP
