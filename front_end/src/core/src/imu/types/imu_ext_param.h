#ifndef __TYPES_IMU_EXT_PARAM_H__
#define __TYPES_IMU_EXT_PARAM_H__

#include <Eigen/Core>
#include <iostream>

namespace front_end{
namespace imu{

struct ImuExtParam
{
    bool use;
    size_t num;
    Eigen::Matrix3d extrinsic_rot;
    Eigen::Matrix3d extrinsic_rpy;
    Eigen::Vector3d extrinsic_trans;

    ImuExtParam();
    
    ImuExtParam(const bool& u, const size_t& n, const Eigen::Matrix3d& ext_rot, 
                const Eigen::Matrix3d& ext_rpy, const Eigen::Vector3d& ext_trans);
};

} // namespace imu
} // namespace front_end

#endif // __TYPES_IMU_EXT_PARAM_H__