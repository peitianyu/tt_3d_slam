#include <iostream>
#include "platform/tt_platform.h"
#include "platform/tt_serialize.h"
#include "common/tt_log.h"
#include "common/tt_sleep.h"
#include "common/tt_test.h"
#include "point.h"

TEST(Platform, Serialize)
{
    std::string topic = "/test";
    platform::Platform::getInstance().CreatePublisher(topic, sizeof(Point));

    Point point(1, 2, 3, 4);
    platform::Serialize serialize;
    serialize << point;

    platform::Platform::getInstance().Publish(topic, serialize.str());

    LOG_DEBUG("Publish: ", point) << std::endl;
}

int main()
{
    RunAllTests();
    return 0;
}