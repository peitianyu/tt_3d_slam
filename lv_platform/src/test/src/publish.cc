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

    // LOG_DEBUG("Publish: ", point) << std::endl;
}

TEST(Platform, Serialize1)
{
    std::string topic = "/test1";
    platform::Platform::getInstance().CreatePublisher(topic, sizeof(Point)*4);

    std::vector<Point> points;
    points.push_back(Point(1, 2, 3, 4));
    points.push_back(Point(2, 3, 4, 5));
    points.push_back(Point(3, 4, 5, 6));
    points.push_back(Point(4, 5, 6, 7));

    platform::Serialize serialize;
    serialize << points;

    platform::Platform::getInstance().Publish(topic, serialize.str());

    // for(auto &point : points)
    //     LOG_DEBUG("Publish: ", point) << std::endl;
}

TEST(Platform, Serialize2)
{
    std::string topic = "/test2";
    platform::Platform::getInstance().CreatePublisher(topic, sizeof(Point)*4);

    std::vector<Point> points;
    points.push_back(Point(1, 2, 3, 4));
    points.push_back(Point(2, 3, 4, 5));
    points.push_back(Point(3, 4, 5, 6));
    points.push_back(Point(4, 5, 6, 7));

    platform::Serialize serialize;
    serialize << points;

    platform::Platform::getInstance().Publish(topic, serialize.str());

    // for(auto &point : points)
    //     LOG_DEBUG("Publish: ", point) << std::endl;
}

int main()
{
    RunAllTests();
    return 0;
}