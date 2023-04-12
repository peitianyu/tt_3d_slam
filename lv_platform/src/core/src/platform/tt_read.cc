#include"tt_read.h"
#include"common/tt_assert.h"
#include"common/tt_sleep.h"
#include"common/tt_crc.h"
#include"common/tt_log.h"

#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <cstring>
#include <iostream>

namespace platform
{


Reader::Reader(std::string topic, size_t max_size):rwlock_(topic), max_size_(max_size + TRANSMIT_SIZE)
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

    rwlock_.ReadUnlock();
}

Reader::~Reader()
{
    // 取消映射共享内存对象
    tt_assert(munmap(shm_ptr_, max_size_) != -1);

    // 关闭共享内存对象和信号量
    tt_assert(close(shm_fd_) != -1);
}

void Reader::Read(std::string &data)
{
    // 加读锁
    rwlock_.ReadLock();

    // 读数据 size + data
    size_t size;
    memcpy(&size, shm_ptr_, sizeof(size_t));

    tt_assert(size + sizeof(size_t) <= max_size_);

    size_t data_size = size - sizeof(uint16_t);
    data.resize(data_size);
    memcpy((char *)data.c_str(), (char *)shm_ptr_ + sizeof(size_t), size);

    // 校验crc
    uint16_t crc;
    memcpy(&crc, (char *)shm_ptr_ + sizeof(size_t) + size, sizeof(uint16_t));

    tt_assert(common::crc16::CheckCRC((uint8_t *)data.c_str(), data.size(), crc));

    // 解读锁
    rwlock_.ReadUnlock();
}

std::string Reader::Read()
{
    std::string data;
    Read(data);
    return data;
}

} // namespace platform
