#include "imu.h"

namespace front_end{
namespace imu{

Imu::Imu(types::Time t, Eigen::Vector3d a, Eigen::Vector3d g)
    : time_stamp(t), acc(a), gyr(g){}

Imu::Imu() : Imu(0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()) {}

} // namespace imu
} // namespace front_end