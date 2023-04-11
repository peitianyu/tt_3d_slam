#ifndef __PLATFORM_READ_H__
#define __PLATFORM_READ_H__

#include"tt_rwlock.h"
#include"tt_define.h"

namespace platform
{

class Reader
{
public:
    Reader(std::string topic, size_t max_size = 10 * MB_1);

    ~Reader();

    void Read(std::string &data);

    std::string Read();
private:
    RWLock rwlock_;
    uint64_t max_size_;
    int shm_fd_;
    void *shm_ptr_;
};

} // namespace platform

#endif  // __PLATFORM_READ_H__