#ifndef __PLATFORM_PUBLISH_H__
#define __PLATFORM_PUBLISH_H__

#include <string>
#include <memory>

#include "tt_write.h"

namespace platform
{

class Publisher
{
public:
    Publisher(std::string topic, size_t max_size);

    void Publish(std::string data);
private:
    std::unique_ptr<Writer> writer_;
};


} // namespace platform

#endif // __PLATFORM_PUBLISH_H__