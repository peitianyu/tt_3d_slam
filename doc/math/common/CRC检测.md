## CRC检测

### 设计思路

```txt
1. 设计一个crc16的检测模块
2. 通过查表法加快检测速度
```

### 程序实现

```c++
uint16_t Update(const uint8_t* data, uint32_t size)
{
    uint16_t crc = 0xffff;
    for (uint32_t i = 0; i < size; ++i)
        crc = (crc >> 8) ^ crc16_tab[(crc ^ data[i]) & 0xff];
    return crc;
}

bool CheckCRC(const uint8_t* data, uint32_t size, uint16_t crc)
{
    return Update(data, size) == crc;
}
```

### 使用方法

```c++
#include"common/tt_crc.h"

TEST(common, crc)
{
    std::string str = "123456789";
    uint16_t crc = common::crc16::Update((uint8_t*)str.c_str(), str.size());
    LOG_DEBUG("crc: ", crc) << std::endl;
    ASSERT_TRUE(common::crc16::CheckCRC((uint8_t*)str.c_str(), str.size(), crc));
}
```
