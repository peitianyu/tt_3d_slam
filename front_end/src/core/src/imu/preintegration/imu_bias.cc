#include"imu_bias.h"

namespace front_end{
namespace imu{

ImuBias::ImuBias(Eigen::Vector3d a, Eigen::Vector3d g)
    : acc_bias(a), gyr_bias(g) {}

ImuBias::ImuBias(Eigen::Matrix<double, 6, 1> bias)
{
    acc_bias = bias.head<3>(0);
    gyr_bias = bias.tail<3>(3);
}

ImuBias::ImuBias(const ImuBias& other)
{
    acc_bias = other.acc_bias;
    gyr_bias = other.gyr_bias;
}

ImuBias::ImuBias() : ImuBias(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()) {}

Eigen::Matrix<double, 6, 1> ImuBias::GetBias() const
{
    Eigen::Matrix<double, 6, 1> bias;
    bias.head<3>(0) = acc_bias;
    bias.tail<3>(3) = gyr_bias;
    return bias;
}

} // namespace imu
} // namespace front_end