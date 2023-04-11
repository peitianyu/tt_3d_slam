#ifndef __IMU_PREINTEGRATOR_H__
#define __IMU_PREINTEGRATOR_H__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <cassert>
#include <map>

#include "../3dr_party/so3.h"
#include "../types/imu.h"
#include "../types/imu_param.h"
#include "imu_jacobian.h"
#include "imu_bias.h"
#include "preintegration_state.h"

namespace front_end{
namespace imu{

class ImuPreintegration
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuPreintegration() = delete;
    
    explicit ImuPreintegration(const ImuParam& param, const ImuBias& init_bias);

    void Propagate(const double& dt, const Imu& imu);

    void Reset();

    void Repropagate();

    void SetState(const PreintegrationState& state) { m_preintegration_state = state; }

    void SetBias(const ImuBias& bias) { m_bias = bias; }

    void SetCovariance(const Eigen::Matrix<double, 15, 15>& covariance) { m_covariance = covariance; }

    const PreintegrationState& State() const { return m_preintegration_state; }

    const ImuBias& Bias() const { return m_bias;}

    const ImuJacbian& Jacbian() const { return m_jacbian; }

    const Eigen::Matrix<double, 15, 15>& Covariance() const { return m_covariance; }

    void Correct(const Eigen::Vector3d& delta_ba, const Eigen::Vector3d& delta_bg);
private:
    ImuParam m_param;
    ImuBias m_bias;
    std::map<double, Imu> m_imu_buf;
    PreintegrationState m_preintegration_state;
    ImuJacbian m_jacbian;
    Eigen::Matrix<double, 15, 15> m_covariance;
};

} // namespace front_end
} // namespace imu

#endif // __IMU_PREINTEGRATOR_H__