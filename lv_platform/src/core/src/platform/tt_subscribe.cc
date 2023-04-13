#include"tt_subscribe.h"

namespace platform
{

Subscriber::Subscriber(std::string topic, size_t max_size, std::function<void(const std::string &)> callback, float period)
{
    reader_ = std::make_unique<Reader>(topic, max_size);
    if(period > 0.0)
        timer_ = std::make_unique<Timer>(period);

    callback_ = callback;
}

void Subscriber::Subscribe()
{
    if (timer_->IsTimeOut()){
        std::string data;
        reader_->Read(data);
        callback_(data);
        timer_->Reset();
    }
}

void Subscriber::SubscribeOnce()
{
    std::string data;
    reader_->Read(data);
    callback_(data);
    timer_->Reset();
}

} // namespace platform

