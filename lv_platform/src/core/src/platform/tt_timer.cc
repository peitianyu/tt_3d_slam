#include"tt_timer.h"
#include <string>

namespace platform
{


Timer::Timer(float period) : period_(period)
{
    Reset();
}

void Timer::Reset()
{
    gettimeofday(&last_time_, NULL);
}

bool Timer::IsTimeOut()
{
    struct timeval now;
    gettimeofday(&now, NULL);
    if ((now.tv_sec - last_time_.tv_sec) + float(now.tv_usec - last_time_.tv_usec) / 1e6 >= period_)
    {
        last_time_ = now;
        return true;
    }
    return false;
}


}
