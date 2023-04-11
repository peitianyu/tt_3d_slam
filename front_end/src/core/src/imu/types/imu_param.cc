#include"imu_param.h"

namespace front_end{
namespace imu{


ImuParam::ImuParam(double sg, double sa, double sgw, double saw, double g)
    : sigma_gyr(sg), sigma_acc(sa), sigma_gyr_walk(sgw), sigma_acc_walk(saw), gravity(g){}

ImuParam::ImuParam() 
    : ImuParam(3.9939570888238808e-03, 1.5636343949698187e-03, 6.4356659353532566e-05, 3.5640318696367613e-05, 9.80511) {}

Eigen::Matrix3d ImuParam::AccCov() const { return sigma_acc * sigma_acc * Eigen::Matrix3d::Identity(); }

Eigen::Matrix3d ImuParam::GyrCov() const { return sigma_gyr * sigma_gyr * Eigen::Matrix3d::Identity(); }

Eigen::Matrix3d ImuParam::AccBiasCov() const { return sigma_acc_walk * sigma_acc_walk * Eigen::Matrix3d::Identity(); }

Eigen::Matrix3d ImuParam::GyrBiasCov() const { return sigma_gyr_walk * sigma_gyr_walk * Eigen::Matrix3d::Identity(); }

Eigen::Matrix<double, 6, 6> ImuParam::NoiseCov() const
{
    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
    cov.block<3, 3>(0, 0) = AccCov();
    cov.block<3, 3>(3, 3) = GyrCov();
    return cov;
}

Eigen::Matrix<double, 6, 6> ImuParam::BiasCov() const
{
    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
    cov.block<3, 3>(0, 0) = AccBiasCov();
    cov.block<3, 3>(3, 3) = GyrBiasCov();
    return cov;
}

} // namespace imu
} // namespace front_end

