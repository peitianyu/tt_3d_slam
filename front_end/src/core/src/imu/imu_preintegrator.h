#ifndef __IMU_PREINTEGRATOR_H__
#define __IMU_PREINTEGRATOR_H__

#include "types/state.h"
#include "imu.h"
#include <Eigen/Dense>

namespace front_end{

class ImuPreintegrator
{
public:
    struct Option
    {
        double acc_noise;
        double gyr_noise;
        double acc_bias_noise;
        double gyr_bias_noise;
        Eigen::Vector3d gravity;

        Option(double acc_noise = 0.01,
               double gyr_noise = 0.001,
               double acc_bias_noise = 0.0001,
               double gyr_bias_noise = 0.0001,
               const Eigen::Vector3d& gravity = Eigen::Vector3d(0, 0, -9.8))
            : acc_noise(acc_noise),
              gyr_noise(gyr_noise),
              acc_bias_noise(acc_bias_noise),
              gyr_bias_noise(gyr_bias_noise),
              gravity(gravity) {}
    };

    ImuPreintegrator(const Option& option) : option_(option) {}

    // 这部分不用于优化，所以不提供雅克比矩阵
    void Preintegrate(const Imu& last_imu, const Imu& cur_imu, types::State& state)
    {
        // Time
        const double delta_t = cur_imu.time_stamp - last_imu.time_stamp;
        const double delta_t2 = delta_t * delta_t;

        // Acc and gyro.
        const Eigen::Vector3d acc = 0.5 * (last_imu.acc + cur_imu.acc) - state.acc_bias;
        const Eigen::Vector3d gyro = 0.5 * (last_imu.gyro + cur_imu.gyro) - state.gyro_bias;

        // Normal state
        // Using P58. of "Quaternion kinematics for the error-state Kalman Filter".
        state.position += delta_t * state.velocity + 0.5 * delta_t2 * (state.orientation * acc + option_.gravity);
        state.velocity += delta_t * (state.orientation * acc + option_.gravity);
        const Eigen::Vector3d delta_angle_axis = gyro * delta_t;
        if (delta_angle_axis.norm() > 1e-12){
            state.orientation = state.orientation * Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix();
        }

        // Covariance of the error-state
        Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * delta_t;
        F.block<3, 3>(3, 6) = -state.orientation * SkewSymmetricMatrix(acc) * delta_t;
        F.block<3, 3>(3, 9) = -state.orientation * delta_t;
        if(delta_angle_axis.norm() > 1e-12){
            F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - SkewSymmetricMatrix(delta_angle_axis);
        }else{
            F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
        }
        F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * delta_t;

        Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
        Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

        Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Zero();
        Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * option_.acc_noise * delta_t2;
        Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * option_.gyr_noise * delta_t2;
        Q.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * option_.acc_bias_noise * delta_t;
        Q.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * option_.gyr_bias_noise * delta_t;

        state.cov = F * state.cov * F.transpose() + Fi * Q * Fi.transpose();
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
    Option option_;
};

} // namespace front_end

#endif // __IMU_PREINTEGRATOR_H__