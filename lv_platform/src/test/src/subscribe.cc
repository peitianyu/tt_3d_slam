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
    
    // LOG_DEBUG("Subscribe: ", point) << std::endl;

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

void Callback1(const std::string &data)
{
    std::vector<Point> points;
    DESERIALIZE(data, points);

    // for(auto &point : points)
    //     LOG_DEBUG("Subscribe: ", point) << std::endl;

    ASSERT_EQ(points[0].x, 1);
    ASSERT_EQ(points[0].y, 2);
    ASSERT_EQ(points[0].z, 3);
    ASSERT_EQ(points[0].intensity, 4);

    ASSERT_EQ(points[1].x, 2);
    ASSERT_EQ(points[1].y, 3);
    ASSERT_EQ(points[1].z, 4);
    ASSERT_EQ(points[1].intensity, 5);

    ASSERT_EQ(points[2].x, 3);
    ASSERT_EQ(points[2].y, 4);
    ASSERT_EQ(points[2].z, 5);
    ASSERT_EQ(points[2].intensity, 6);

    ASSERT_EQ(points[3].x, 4);
    ASSERT_EQ(points[3].y, 5);
    ASSERT_EQ(points[3].z, 6);
    ASSERT_EQ(points[3].intensity, 7);
}

TEST(Platform, Deserialize1)
{
    std::string topic = "/test1";
    platform::Platform::getInstance().CreateSubscriber(topic, sizeof(Point)*4, Callback1, 1.0);

    // platform::Platform::getInstance().Spin();
    platform::Platform::getInstance().SpinOnce();
}

TEST(Platform, Deserialize3)
{
    std::string topic = "/test2";
    platform::Platform::getInstance().CreateSubscriber(topic, sizeof(Point)*2, Callback1, 1.0);

    // platform::Platform::getInstance().Spin();
    platform::Platform::getInstance().SpinOnce();
}


int main()
{
    RunAllTests();
    return 0;
}