#ifndef __PLATFORM_RWLOCK_H__
#define __PLATFORM_RWLOCK_H__

#include <string>
#include <semaphore.h>
#include <fcntl.h>

namespace platform
{

class RWLock {
public:
    RWLock(std::string topic);

    ~RWLock();

    void ReadLock();

    void ReadUnlock();

    void WriteLock();

    void WriteUnlock();
private:
    sem_t *sem_read_ptr_;
    sem_t *sem_write_ptr_;
};



} // namespace platform






#endif // __PLATFORM_RWLOCK_H__