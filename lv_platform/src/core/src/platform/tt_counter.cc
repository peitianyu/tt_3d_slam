#include"tt_counter.h"


namespace platform
{

Counter::Counter(int count) : count_(count), current_count_(0)
{}

void Counter::Reset() { current_count_ = 0; }


bool Counter::IsCountOut()
{
    ++current_count_;
    if (current_count_ >= count_){
        return true;
    }
    return false;
}

} // namespace platform