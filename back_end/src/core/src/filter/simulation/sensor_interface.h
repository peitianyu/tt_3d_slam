#ifndef __SENSOR_INTERFACE_H__
#define __SENSOR_INTERFACE_H__

#include "gps/gps.h"
#include "imu/imu.h"


namespace simulation
{

void RegisterGpsCallback(void(*cb)(const front_end::Gps&));

void RegisterImuCallback(void(*cb)(const front_end::Imu&));

} // namespace simulation

#endif // __SENSOR_INTERFACE_H__