#ifndef __TYPES_IMU_BIAS_H__
#define __TYPES_IMU_BIAS_H__

#include <Eigen/Core>

namespace front_end{
namespace imu{


struct ImuBias
{
    Eigen::Vector3d acc_bias;
    Eigen::Vector3d gyr_bias;

    Eigen::Matrix<double, 6, 1> GetBias() const;

    ImuBias(Eigen::Vector3d a, Eigen::Vector3d g);

    ImuBias(Eigen::Matrix<double, 6, 1> bias);

    ImuBias(const ImuBias& other);

    ImuBias();
};

} // namespace imu
} // namespace front_end

#endif // __TYPES_IMU_BIAS_H__