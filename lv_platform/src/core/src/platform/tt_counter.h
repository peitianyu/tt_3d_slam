#ifndef __PLATFORM_COUNTER_H__
#define __PLATFORM_COUNTER_H__

namespace platform
{

class Counter
{
public:
    Counter(int count = 1);

    void Reset();

    bool IsCountOut();
private:
    int count_;
    int current_count_;
};

}

#endif // __PLATFORM_COUNTER_H__