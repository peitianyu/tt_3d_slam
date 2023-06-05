#ifndef __TYPES_STATE_H__
#define __TYPES_STATE_H__

#include <Eigen/Core>
#include <memory>
#include "time.h"

namespace types {

struct State
{
    Time time_stamp;

    // State.
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Matrix3d orientation;
    Eigen::Vector3d acc_bias;
    Eigen::Vector3d gyro_bias;

    // Covariance.
    Eigen::Matrix<double, 15, 15> cov;

    State() : time_stamp(0),
              position(Eigen::Vector3d::Zero()),
              velocity(Eigen::Vector3d::Zero()),
              orientation(Eigen::Matrix3d::Identity()),
              acc_bias(Eigen::Vector3d::Zero()),
              gyro_bias(Eigen::Vector3d::Zero()),
              cov(Eigen::Matrix<double, 15, 15>::Zero()) {}
};

}

#endif // __TYPES_STATE_H__