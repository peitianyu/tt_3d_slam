#ifndef __TYPES_IMU_PREINTEGRATION_STATE_H__
#define __TYPES_IMU_PREINTEGRATION_STATE_H__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "types/time.h"

namespace front_end{
namespace imu{

struct PreintegrationState
{
    lv_math::types::Time sum_dt;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Matrix3d rot;

    Eigen::Matrix<double, 9, 1> GetState() const;
                       
    PreintegrationState(lv_math::types::Time dt = 0.0, Eigen::Vector3d p = Eigen::Vector3d::Zero(), 
                        Eigen::Vector3d v = Eigen::Vector3d::Zero(), Eigen::Vector3d r = Eigen::Vector3d::Zero());
    
    PreintegrationState(lv_math::types::Time dt, Eigen::Matrix<double, 9, 1> state);

    PreintegrationState(const PreintegrationState& other);
};

} // namespace imu
} // namespace front_end

#endif // __TYPES_IMU_PREINTEGRATION_STATE_H__