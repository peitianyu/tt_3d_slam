#ifndef __PLATFORM_TIMER_H__
#define __PLATFORM_TIMER_H__

#include <sys/time.h>

namespace platform
{

class Timer 
{
public:
    Timer(float period = 1.0);

    void Reset();

    bool IsTimeOut();
private:
    float period_; // 间隔时间: 单位秒
    struct timeval last_time_;
};

}

#endif // __PLATFORM_TIMER_H__