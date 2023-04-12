#include <iostream>
#include "platform/tt_platform.h"
#include "platform/tt_deserialize.h"
#include "common/tt_test.h"
#include "common/tt_log.h"
#include "point.h"

#define DESERIALIZE(data, ...)               \
    platform::Deserialize deserialize(data); \
    deserialize >> __VA_ARGS__;

void Callback(const std::string &data)
{
    Point point;
    DESERIALIZE(data, point);
    
    LOG_DEBUG("Subscribe: ", point) << std::endl;

    ASSERT_EQ(point.x, 1);
    ASSERT_EQ(point.y, 2);
    ASSERT_EQ(point.z, 3);
    ASSERT_EQ(point.intensity, 4);
}

TEST(Platform, Deserialize)
{
    std::string topic = "/test";
    platform::Platform::getInstance().CreateSubscriber(topic, sizeof(Point), Callback, 1.0);

    // platform::Platform::getInstance().Spin();
    platform::Platform::getInstance().SpinOnce();
}

int main()
{
    RunAllTests();
    return 0;
}