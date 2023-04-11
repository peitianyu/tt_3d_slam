#ifndef __PLATFORM_WRITE_H__
#define __PLATFORM_WRITE_H__

#include"tt_rwlock.h"
#include"tt_define.h"

namespace platform
{

class Writer
{
public:
    Writer(std::string topic, uint64_t max_size = 10 * MB_1);

    ~Writer();

    void Write(std::string data);
private:
    RWLock rwlock_;
    uint64_t max_size_;
    int shm_fd_;
    void *shm_ptr_;
};

} // namespace platform

#endif  // __PLATFORM_WRITE_H__