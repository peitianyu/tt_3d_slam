#include"tt_sleep.h"
#include <unistd.h>

namespace common
{

void Sleep(float seconds)
{
    usleep(seconds * 1e6);
}

void RateSleep(float rate)
{
    usleep(1e6 / rate);
}

} // namespace common
