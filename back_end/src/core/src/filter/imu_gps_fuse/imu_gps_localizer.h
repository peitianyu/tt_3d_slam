#ifndef __IMU_GPS_LOCALIZER_H__
#define __IMU_GPS_LOCALIZER_H__

#include <memory>

#include "types/state.h"
#include "gps/gps.h"
#include "gps/gps_convertor.h"
#include "imu/imu.h"
#include "imu/imu_preintegrator.h"

// 外参标定: extrinsic parameters calibration
class ImuGpsLocalizer
{
public:
    ImuGpsLocalizer(const Eigen::Vector3d& calib_extr, const front_end::ImuPreintegrator::Option& imu_option)
        : is_inited_(false), calib_extr_(calib_extr), fuse_state_(types::State()),
          imu_preintegrator_(std::unique_ptr<front_end::ImuPreintegrator>(new front_end::ImuPreintegrator(imu_option))){}
          

    bool ImuPredict(const front_end::Imu& imu)
    {
        if(!is_inited_) return false;

        static front_end::Imu last_imu = imu;

        imu_preintegrator_->Preintegrate(last_imu, imu, fuse_state_);

        last_imu = imu;
        return true;
    }

    void GpsUpdate(const front_end::Gps& gps)
    {
        // FIXME: 实际上还有一个时间戳对齐, 这里暂时不考虑
        if(!is_inited_){
            is_inited_ = true;
            gps_convertor_ = std::unique_ptr<front_end::GpsConverter>(new front_end::GpsConverter(gps.lla));
            return ;
        }

        UpdateStateByGps(gps);
    }

    types::State GetFuseState() { return fuse_state_; }
private:
    void UpdateStateByGps(const front_end::Gps& gps)
    {
        Eigen::Matrix<double, 3, 15> jacobian;
        Eigen::Vector3d residual;
        ComputeJacobianAndResidual(gps, fuse_state_, gps_convertor_, jacobian, residual);
        Eigen::Matrix3d cov = gps.cov;

        // EKF
        Eigen::MatrixXd P = fuse_state_.cov;
        Eigen::MatrixXd K = P * jacobian.transpose() * (jacobian * P * jacobian.transpose() + cov).inverse();
        Eigen::VectorXd delta_x = K * residual;

        // Update state.
        UpdateState(delta_x, fuse_state_);

        // Update covariance.
        Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * jacobian;
        fuse_state_.cov = I_KH * P * I_KH.transpose() + K * cov * K.transpose();
    }

    void ComputeJacobianAndResidual(const front_end::Gps& gps, const types::State& state, const std::unique_ptr<front_end::GpsConverter>& gps_convertor, 
                                    Eigen::Matrix<double, 3, 15>& jacobian, Eigen::Vector3d& residual)
    {
        // convert wgs84 to enu
        Eigen::Vector3d gps_pos = gps_convertor_->LLAToENU(gps.lla);

        // compute residual
        residual = gps_pos - (state.position + state.orientation * calib_extr_);

        // compute jacobian
        jacobian.setZero();
        jacobian.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        jacobian.block<3, 3>(0, 6) = -state.orientation * SkewSymmetricMatrix(calib_extr_);
    }

    void UpdateState(const Eigen::VectorXd& delta_x, types::State& state)
    {
        state.position += delta_x.segment<3>(0);
        state.velocity += delta_x.segment<3>(3);
        state.acc_bias += delta_x.segment<3>(9);
        state.gyro_bias += delta_x.segment<3>(12);

        if (delta_x.block<3, 1>(6, 0).norm() > 1e-12)
            state.orientation *= Eigen::AngleAxisd(delta_x.block<3, 1>(6, 0).norm(), delta_x.block<3, 1>(6, 0).normalized()).toRotationMatrix();
    }
private:
    Eigen::Matrix3d SkewSymmetricMatrix(const Eigen::Vector3d& vec)
    {
        Eigen::Matrix3d skew_symmetric_matrix;
        skew_symmetric_matrix << 0, -vec(2), vec(1),
                                 vec(2), 0, -vec(0),
                                 -vec(1), vec(0), 0;
        return skew_symmetric_matrix;
    }
private:
    bool is_inited_;
    Eigen::Vector3d calib_extr_;
    types::State fuse_state_;
    std::unique_ptr<front_end::ImuPreintegrator> imu_preintegrator_;
    std::unique_ptr<front_end::GpsConverter> gps_convertor_;
};


#endif // __IMU_GPS_LOCALIZER_H__