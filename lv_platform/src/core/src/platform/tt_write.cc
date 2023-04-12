#include "tt_write.h"
#include "common/tt_assert.h"
#include "common/tt_sleep.h"
#include "common/tt_crc.h"
#include "common/tt_log.h"

#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <cstring>
#include <iostream>

namespace platform
{

Writer::Writer(std::string topic, uint64_t max_size):rwlock_(topic), max_size_(max_size + TRANSMIT_SIZE)
{
    // 创建或打开共享内存对象
    shm_fd_ = shm_open((topic + "_shm").c_str(), O_CREAT | O_RDWR, 0666);
    tt_assert(shm_fd_ != -1);

    // 设置共享内存对象的大小
    if (ftruncate(shm_fd_, max_size) == -1)
    tt_assert(ftruncate(shm_fd_, max_size) != -1);

    // 将共享内存对象映射到进程的虚拟地址空间
    shm_ptr_ = mmap(NULL, max_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    tt_assert(shm_ptr_ != MAP_FAILED);

    rwlock_.WriteUnlock();
}

Writer::~Writer()
{
    // 取消映射共享内存对象
    tt_assert(munmap(shm_ptr_, max_size_) != -1);

    // 关闭共享内存对象和信号量
    tt_assert(close(shm_fd_) != -1);
}

void Writer::Write(std::string data)
{
    // 加写锁
    rwlock_.WriteLock();

    if(data.size() > max_size_){
        LOG_ERROR("data size is too large, data size: ", data.size(), " max size: ", max_size_) << std::endl;
    }

    // 写数据 size + data
    size_t size = data.size() + sizeof(uint16_t);
    memcpy(shm_ptr_, &size, sizeof(size_t));
    memcpy((char *)shm_ptr_ + sizeof(size_t), data.c_str(), size);
    uint16_t crc = common::crc16::Update((uint8_t *)data.c_str(), data.size());
    memcpy((char *)shm_ptr_ + sizeof(size_t) + size, &crc, sizeof(uint16_t));

    // 解写锁
    rwlock_.WriteUnlock();
}

} // namespace platform
