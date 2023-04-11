#include "imu_jacobian.h"

namespace front_end{
namespace imu{


ImuJacbian::ImuJacbian(Eigen::Matrix3d dpos_gyr, Eigen::Matrix3d dpos_acc, Eigen::Matrix3d dvel_gyr, Eigen::Matrix3d dvel_acc, Eigen::Matrix3d drot_gyr)
    : dpos_gyr(dpos_gyr), dpos_acc(dpos_acc), dvel_gyr(dvel_gyr), dvel_acc(dvel_acc), drot_gyr(drot_gyr) {}

ImuJacbian::ImuJacbian() 
    : ImuJacbian(Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), 
        Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero()) {}


} // namespace imu
} // namespace front_end
