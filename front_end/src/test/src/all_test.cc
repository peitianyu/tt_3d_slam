#include<iostream>
#include"common/tt_log.h"
#include"common/tt_test.h"


TEST(GnssTest, Test)
{
    LOG_DEBUG("Hello World!") << std::endl;
}


int main()
{
    return RunAllTests();
}