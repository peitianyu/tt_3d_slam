#include "imu_ext_param.h"

namespace front_end{
namespace imu{

ImuExtParam::ImuExtParam()
{
    use = false;
    num = 2000;
    extrinsic_rot << 1,0,0,0,1,0,0,0,-1;
    extrinsic_rpy << 1,0,0,0,1,0,0,0,1;
    extrinsic_trans << 0,0,0;
}

ImuExtParam::ImuExtParam(const bool& u, const size_t& n, const Eigen::Matrix3d& ext_rot, const Eigen::Matrix3d& ext_rpy, const Eigen::Vector3d& ext_trans)
    : use(u), num(n), extrinsic_rot(ext_rot), extrinsic_rpy(ext_rpy), extrinsic_trans(ext_trans) {}


} // namespace imu
} // namespace front_end