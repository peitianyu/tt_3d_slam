#ifndef __TYPES_IMU_PARAM_H__
#define __TYPES_IMU_PARAM_H__

#include <Eigen/Core>
#include <iostream>

namespace front_end{
namespace imu{

struct ImuParam
{
    double sigma_gyr;
    double sigma_acc;
    double sigma_gyr_walk;
    double sigma_acc_walk;
    double gravity;

    ImuParam(double sg, double sa, double sgw, double saw, double g);
    
    ImuParam();

    Eigen::Matrix3d AccCov() const;

    Eigen::Matrix3d GyrCov() const;

    Eigen::Matrix3d AccBiasCov() const;

    Eigen::Matrix3d GyrBiasCov() const;

    Eigen::Matrix<double, 6, 6> NoiseCov() const;

    Eigen::Matrix<double, 6, 6> BiasCov() const;
};

} // namespace imu
} // namespace front_end

#endif // __TYPES_IMU_PARAM_H__