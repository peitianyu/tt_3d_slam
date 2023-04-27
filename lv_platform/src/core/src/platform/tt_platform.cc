#include"tt_platform.h"
#include"common/tt_sleep.h"

namespace platform
{

void Platform::CreatePublisher(std::string topic, size_t max_size)
{
    auto it = m_publishers.find(topic);
    if (it == m_publishers.end())
        m_publishers[topic] = std::make_unique<Publisher>(topic, max_size);
}

void Platform::CreateSubscriber(
    std::string topic, size_t max_size,
    std::function<void(const std::string &)> callback,
    float period)
{
    if (m_subscribers.find(topic) == m_subscribers.end())
        m_subscribers[topic] = std::make_unique<Subscriber>(topic, max_size, callback, period);
}

void Platform::Publish(std::string topic, const std::string &data)
{
    auto it = m_publishers.find(topic);
    if (it != m_publishers.end())
        static_cast<Publisher *>(it->second.get())->Publish(data);
}

void Platform::Spin()
{
    while (true){
        for (auto &it : m_subscribers)
            static_cast<Subscriber *>(it.second.get())->Subscribe();
        
        common::Sleep(0.001);
    }
}

void Platform::SpinOnce()
{
    for (auto &it : m_subscribers)
        static_cast<Subscriber *>(it.second.get())->SubscribeOnce();
}

} // namespace platform

