#ifndef __TEST_SINGLETON_H__
#define __TEST_SINGLETON_H__

#include"common/tt_singleton.h"
#include"common/tt_log.h"

namespace test
{

class TestSingleton
{
public:
    static TestSingleton* GetInstance()
    {
        static Singleton<TestSingleton> s_instance;
        return s_instance.Get();
    }

    void Print()
    {
        
        LOG_DEBUG("This is a debug log") << std::endl;
    }

    std::string m_str = "test_singleton";
};


}// namespace test




#endif // __TEST_SINGLETON_H__