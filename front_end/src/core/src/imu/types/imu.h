#ifndef __TYPES_IMU_H__
#define __TYPES_IMU_H__

#include <Eigen/Core>
#include "types/time.h"

namespace front_end{
namespace imu{
    
struct Imu
{
    lv_math::types::Time time_stamp;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;

    Imu(lv_math::types::Time t, Eigen::Vector3d a, Eigen::Vector3d g);

    Imu();
};

} // namespace imu
} // namespace front_end


#endif // __TYPES_IMU_H__