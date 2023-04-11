#include <iostream>
#include "platform/tt_platform.h"
#include "platform/tt_serialize.h"
#include "common/tt_log.h"
#include "common/tt_sleep.h"
#include "point.h"



void Test()
{
    Point point(1, 2, 3, 4);
    std::string topic = "/test";
    platform::Platform::getInstance().CreatePublisher(topic, sizeof(Point));

    platform::Serialize serialize;
    serialize << point;

    platform::Platform::getInstance().Publish(topic, serialize.str());

    LOG_DEBUG("Publish: ", point) << std::endl;

    common::RateSleep(2.0);
}

int main()
{
    Test();
    return 0;
}