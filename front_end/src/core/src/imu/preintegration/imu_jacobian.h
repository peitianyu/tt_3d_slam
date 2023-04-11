#ifndef __TYPES_IMU_JACOBIAN_H__
#define __TYPES_IMU_JACOBIAN_H__

#include <Eigen/Core>

namespace front_end{
namespace imu{


struct ImuJacbian
{
    Eigen::Matrix3d dpos_gyr;
    Eigen::Matrix3d dpos_acc;
    Eigen::Matrix3d dvel_gyr;
    Eigen::Matrix3d dvel_acc;
    Eigen::Matrix3d drot_gyr;

    ImuJacbian(Eigen::Matrix3d dpos_gyr, Eigen::Matrix3d dpos_acc, 
        Eigen::Matrix3d dvel_gyr, Eigen::Matrix3d dvel_acc, Eigen::Matrix3d drot_gyr);

    ImuJacbian();
};

} // namespace imu
} // namespace front_end

#endif // __TYPES_IMU_JACOBIAN_H__