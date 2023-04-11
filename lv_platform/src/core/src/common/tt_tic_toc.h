#ifndef __COMMON_TEST_H__
#define __COMMON_TEST_H__

#include <sys/time.h>

namespace common
{

class TicToc
{
public:
    TicToc();

    void Tic();

    float Toc();
private:
    struct timeval last_time_;
};


}

#endif // __COMMON_TIC_TOC_H__