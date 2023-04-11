#include "tt_rwlock.h"
#include "common/tt_assert.h"

namespace platform
{

RWLock::RWLock(std::string topic){
    // 创建或打开信号量
    sem_read_ptr_ = sem_open((topic + "_read").c_str(), O_CREAT, 0666, 1);
    tt_assert(sem_read_ptr_ != SEM_FAILED);

    sem_write_ptr_ = sem_open((topic + "_write").c_str(), O_CREAT, 0666, 1);
    tt_assert(sem_write_ptr_ != SEM_FAILED);
}

RWLock::~RWLock() {
    tt_assert(sem_close(sem_read_ptr_) != -1);

    tt_assert(sem_close(sem_write_ptr_) != -1);
}

void RWLock::ReadLock() {
    // 等待信号量的值大于零，然后将其减一
    tt_assert(sem_wait(sem_read_ptr_) != -1);

    // 等待信号量的值大于零，然后将其减一
    tt_assert(sem_wait(sem_write_ptr_) != -1);

    // 将信号量的值加一
    tt_assert(sem_post(sem_write_ptr_) != -1);
}

void RWLock::ReadUnlock() {
    // 将信号量的值加一
    tt_assert(sem_post(sem_read_ptr_) != -1);
}

void RWLock::WriteLock() {
    // 等待信号量的值大于零，然后将其减一
    tt_assert(sem_wait(sem_write_ptr_) != -1);
}

void RWLock::WriteUnlock() {
    // 将信号量的值加一
    tt_assert(sem_post(sem_write_ptr_) != -1);
}

} // namespace platform




