#include <iostream>
#include "platform/tt_platform.h"
#include "platform/tt_deserialize.h"
#include "common/tt_test.h"
#include "common/tt_log.h"
#include "point.h"

void Callback(const std::string &data)
{
    platform::Deserialize deserialize(data);
    Point point;
    deserialize >> point;
    
    LOG_INFO("Subscribe: ", point) << std::endl;
    LOG_WARN("Subscribe: ", point) << std::endl;
    LOG_DEBUG("Subscribe: ", point) << std::endl;
    LOG_ERROR("Subscribe: ", point) << std::endl;

    ASSERT_EQ(point.x, 1);
    ASSERT_EQ(point.y, 2);
    ASSERT_EQ(point.z, 3);
    ASSERT_EQ(point.intensity, 2); // fail
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