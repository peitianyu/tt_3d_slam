#ifndef __PLATFORM_PLATFORM_H__
#define __PLATFORM_PLATFORM_H__

#include"tt_publish.h"
#include"tt_subscribe.h"

#include<map>
#include<memory>
#include<functional>

namespace platform
{

class Platform
{
public:
    static Platform &getInstance(){
        static Platform instance;
        return instance;
    }

    void CreatePublisher(std::string topic, size_t max_size);

    void CreateSubscriber(
        std::string topic, size_t max_size, 
        std::function<void(const std::string &)> callback, 
        float period = -1.0);

    void Publish(std::string topic, const std::string& data);

    void Spin();

    void SpinOnce();
private:
    Platform() {}
    Platform(const Platform &) = delete;
    Platform &operator=(const Platform &) = delete;

    std::map<std::string, std::unique_ptr<Publisher>> m_publishers;
    std::map<std::string, std::unique_ptr<Subscriber>> m_subscribers;
};


} // namespace platform

#endif // __PLATFORM_PLATFORM_H__