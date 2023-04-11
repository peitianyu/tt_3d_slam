#include"tt_publish.h"

namespace platform
{

Publisher::Publisher(std::string topic, size_t max_size)
{
    writer_ = std::make_unique<Writer>(topic, max_size);
}

void Publisher::Publish(std::string data)
{
    writer_->Write(data);
}

} // namespace platform

