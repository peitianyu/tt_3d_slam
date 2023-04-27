#include<iostream>
#include"common/tt_test.h"
#include"common/tt_log.h"
#include"common/tt_sleep.h"
#include"common/tt_tic_toc.h"

#include"platform/tt_serialize.h"
#include"platform/tt_deserialize.h"
#include"platform/tt_timer.h"
#include"platform/tt_counter.h"

#define DESERIALIZE(name, data, ...)                        \
    platform::Deserialize deserialize_##name(data);         \
    deserialize_##name >> __VA_ARGS__;

TEST(all_test, serialize)
{
    std::string str = "hello world";

    platform::Serialize serialize;
    serialize << str;

    std::string str1 = serialize.str();
    std::string str2;
    DESERIALIZE(1, str1, str1);
    ASSERT_EQ(str, str1);

    std::vector<std::string> strs = {"hello", "world", "!"};
    platform::Serialize serialize1;
    serialize1 << strs;

    std::string str3 = serialize1.str();
    std::vector<std::string> strs1;
    DESERIALIZE(2, str3, strs1);
    for(uint i = 0; i < strs.size(); i++)
        ASSERT_EQ(strs[i], strs1[i]);
}

TEST(all_test, timer)
{
    common::TicToc tic_toc;
    platform::Timer timer(1.0);
    timer.Reset();
    tic_toc.Tic();
    common::Sleep(1.5);

    bool is_time_out = timer.IsTimeOut();
    LOG_DEBUG("time: ", tic_toc.Toc(), is_time_out) << std::endl;
    ASSERT_TRUE(is_time_out);
    tic_toc.Tic();
    common::Sleep(0.5);
    is_time_out = timer.IsTimeOut();
    LOG_DEBUG("time: ", tic_toc.Toc(), is_time_out) << std::endl;
    ASSERT_FALSE(is_time_out);

    tic_toc.Tic();
    uint cnt = 3;
    while(cnt--)
    {
        if(timer.IsTimeOut())
        {
            LOG_DEBUG("time out! tic_toc: ", tic_toc.Toc()) << std::endl;
            timer.Reset();
        }
        common::Sleep(0.1);
    }
}

TEST(all_test, counter)
{
    platform::Counter counter(3);
    counter.Reset();
    ASSERT_FALSE(counter.IsCountOut());
    ASSERT_FALSE(counter.IsCountOut());
    ASSERT_TRUE(counter.IsCountOut());
    counter.Reset();
    ASSERT_FALSE(counter.IsCountOut());
    ASSERT_FALSE(counter.IsCountOut());
    ASSERT_TRUE(counter.IsCountOut());
}


int main()
{
    return RunAllTests();
}