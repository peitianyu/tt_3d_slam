#ifndef __COMMON_CRC_H__
#define __COMMON_CRC_H__

#include <stdint.h>

namespace common{
namespace crc16
{

uint16_t Update(const uint8_t* data, uint32_t size);

bool CheckCRC(const uint8_t* data, uint32_t size, uint16_t crc);

}
} // namespace common

#endif // __COMMON_CRC_H__