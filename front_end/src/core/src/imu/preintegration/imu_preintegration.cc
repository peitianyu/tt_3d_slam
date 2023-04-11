#include"imu_preintegration.h"

namespace front_end{
namespace imu{

ImuPreintegration::ImuPreintegration(const ImuParam& param, const ImuBias& init_bias)
    : m_param(param), m_bias(init_bias) 
{
    m_preintegration_state = PreintegrationState();
    m_jacbian = ImuJacbian();
    m_covariance = Eigen::Matrix<double, 15, 15>::Zero();
}

void ImuPreintegration::Propagate(const double& dt, const Imu& imu)
{
    assert(dt > 0);

    m_imu_buf[dt] = imu;
    
    Eigen::Vector3d acc = imu.acc - m_bias.acc_bias - Eigen::Vector3d(0, 0, m_param.gravity);
    Eigen::Vector3d gyr = imu.gyr - m_bias.gyr_bias;

    double dt2 = dt * dt;
    Eigen::Matrix3d i3 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d hat_acc = Sophus::SO3::hat(acc);
    Eigen::Matrix3d dR = Sophus::SO3::exp(gyr * dt).matrix();
    Eigen::Matrix3d Jr_R = Sophus::SO3::JacobianR(gyr * dt);

    // Update covariance
    Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Identity();
    A.block<3, 3>(3, 6) = -dt * m_preintegration_state.rot * hat_acc;
    A.block<3, 3>(0, 6) = -0.5 * dt2 * m_preintegration_state.rot * hat_acc;
    A.block<3, 3>(6, 6) = dR.transpose();
    A.block<3, 3>(0, 3) = i3 * dt;
    
    Eigen::Matrix<double, 9, 3> Bg = Eigen::Matrix<double, 9, 3>::Zero();
    Bg.block<3, 3>(6, 0) = Jr_R * dt;

    Eigen::Matrix<double, 9, 3> Ba = Eigen::Matrix<double, 9, 3>::Zero();
    Ba.block<3, 3>(0, 0) = dt * m_preintegration_state.rot;
    Ba.block<3, 3>(3, 0) = 0.5 * dt2 * m_preintegration_state.rot;

    m_covariance.block<9, 9>(0, 0) = A * m_covariance.block<9, 9>(0, 0) * A.transpose() + 
                                        Bg * m_param.GyrCov() * Bg.transpose() + 
                                        Ba * m_param.AccCov() * Ba.transpose();
    m_covariance.block<6, 6>(9, 9) = m_param.BiasCov();

    // Update Jacobian
    m_jacbian.dpos_acc += dt * m_jacbian.dvel_acc - 0.5 * dt2 * m_preintegration_state.rot;
    m_jacbian.dpos_gyr += m_jacbian.dpos_gyr * dt - 0.5 * dt2 * m_preintegration_state.rot * hat_acc * m_jacbian.drot_gyr;
    
    m_jacbian.dvel_acc += -dt * m_preintegration_state.rot;
    m_jacbian.dvel_gyr += -dt * m_preintegration_state.rot * hat_acc * m_jacbian.drot_gyr;
    
    m_jacbian.drot_gyr += dR.transpose() * m_jacbian.drot_gyr - Jr_R * dt;

    // Update state
    m_preintegration_state.sum_dt += dt;
    m_preintegration_state.pos += m_preintegration_state.vel * dt + 0.5 * m_preintegration_state.rot * acc * dt2;
    m_preintegration_state.vel += m_preintegration_state.rot * acc * dt;
    m_preintegration_state.rot = m_preintegration_state.rot * dR;
}

void ImuPreintegration::Reset()
{
    m_preintegration_state = PreintegrationState();
    m_jacbian = ImuJacbian();
    m_covariance = Eigen::Matrix<double, 15, 15>::Zero();
}

void ImuPreintegration::Repropagate()
{
    Reset();
    for (auto& imu : m_imu_buf)
        Propagate(imu.first, imu.second);
}

void ImuPreintegration::Correct(const Eigen::Vector3d& delta_ba, const Eigen::Vector3d& delta_bg)
{
    m_preintegration_state.rot = m_preintegration_state.rot * Sophus::SO3::exp(m_jacbian.drot_gyr * delta_bg).matrix();
    m_preintegration_state.vel += m_jacbian.dvel_acc * delta_ba +  m_jacbian.dvel_gyr * delta_bg;
    m_preintegration_state.pos += m_jacbian.dpos_acc * delta_ba + m_jacbian.dpos_gyr * delta_bg;
}


} // namespace front_end
} // namespace imu