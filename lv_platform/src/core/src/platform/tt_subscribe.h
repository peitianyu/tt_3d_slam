#ifndef __PLATFORM_SUBSCRIBE_H__
#define __PLATFORM_SUBSCRIBE_H__


#include <string>
#include <functional>
#include <memory>

#include "tt_timer.h"
#include "tt_read.h"

namespace platform
{

class Subscriber
{
public:
    Subscriber(
        std::string topic, size_t max_size,
        std::function<void(const std::string &)> callback,
        float period = -1.0);

    void Subscribe();

    void SubscribeOnce();
private:
    std::unique_ptr<Reader> reader_;
    std::unique_ptr<Timer> timer_;
    std::function <void(const std::string&)> callback_;
};


} // namespace platform


#endif // __PLATFORM_SUBSCRIBE_H__