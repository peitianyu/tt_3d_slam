#include<iostream>
#include"common/tt_test.h"
#include"common/tt_log.h"

TEST(Common, Test)
{
    ASSERT_EQ(1, 1);
    ASSERT_EQ(1.2, 1.2);
}

TEST(Common, Log)
{
    LOG_INFO("This is a info log") << std::endl;
    LOG_DEBUG("This is a debug log") << std::endl;
    LOG_WARN("This is a warn log") << std::endl;
    LOG_ERROR("This is a error log") << std::endl;


    LOG_INFO("This is a info log", 1, 2, 3) << std::endl;
}

int main()
{
    return RunAllTests();
}