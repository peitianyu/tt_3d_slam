#include"tt_tic_toc.h"
#include <string>

namespace common
{

TicToc::TicToc()
{
    Tic();
}

void TicToc::Tic()
{
    gettimeofday(&last_time_, NULL);
}

float TicToc::Toc()
{
    struct timeval now;
    gettimeofday(&now, NULL);
    return (now.tv_sec - last_time_.tv_sec) + float(now.tv_usec - last_time_.tv_usec) / 1e6;
}


}
