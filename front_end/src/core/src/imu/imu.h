#ifndef __IMU_H__
#define __IMU_H__

#include <Eigen/Core>
#include <memory>

#include "types/time.h"

namespace front_end{

struct Imu
{
    types::Time time_stamp;

    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};

} // namespace front_end

#endif // __IMU_H__