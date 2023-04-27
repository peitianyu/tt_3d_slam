#include "tt_rwlock.h"
#include "common/tt_assert.h"

namespace platform
{

RWLock::RWLock(std::string topic){
    sem_read_ptr_ = sem_open((topic + "_read").c_str(), O_CREAT, 0666, 1);
    tt_assert(sem_read_ptr_ != SEM_FAILED);

    sem_write_ptr_ = sem_open((topic + "_write").c_str(), O_CREAT, 0666, 1);
    tt_assert(sem_write_ptr_ != SEM_FAILED);

    ReadUnlock();
    WriteUnlock();
}

RWLock::~RWLock() {
    tt_assert(sem_close(sem_read_ptr_) != -1);

    tt_assert(sem_close(sem_write_ptr_) != -1);
}

void RWLock::ReadLock() {
    tt_assert(sem_wait(sem_read_ptr_) != -1);

    tt_assert(sem_wait(sem_write_ptr_) != -1);

    tt_assert(sem_post(sem_write_ptr_) != -1);
}

void RWLock::ReadUnlock() {
    tt_assert(sem_post(sem_read_ptr_) != -1);
}

void RWLock::WriteLock() {
    tt_assert(sem_wait(sem_write_ptr_) != -1);
}

void RWLock::WriteUnlock() {
    tt_assert(sem_post(sem_write_ptr_) != -1);
}

} // namespace platform




