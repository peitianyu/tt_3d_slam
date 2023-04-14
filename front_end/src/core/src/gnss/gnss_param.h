#ifndef __GNSS_PARAM_H__
#define __GNSS_PARAM_H__

#include<Eigen/Core>
#include"types/pose3d.h"

namespace front_end{
namespace gnss{

struct GnssParam
{
    enum GpsStatusType
    {
        GNSS_FLOAT_SOLUTION = 5,        // 浮点解（cm到dm之间）
        GNSS_FIXED_SOLUTION = 4,        // 固定解（cm级）
        GNSS_PSEUDO_SOLUTION = 2,       // 伪距差分解（分米级）
        GNSS_SINGLE_POINT_SOLUTION = 1, // 单点解（10m级）
        GNSS_NOT_EXIST = 0,             // GPS无信号
        GNSS_OTHER = -1,                // 其他
    };

    bool using_heading;
    double trans_noise;
    double rot_noise;
    Eigen::Vector3d antenna_offset; // (x, y, theta)(m, m, rad)
    GpsStatusType status;
    
    types::Pose3D GetAntennaPose() const;
    Eigen::Matrix3d GetTransCov() const;
    Eigen::Matrix3d GetRotCov() const;

    GnssParam(bool u, double t_n, double r_n, Eigen::Vector3d a, GpsStatusType s);
   
    GnssParam(); // 这里之后结合ParamServer进行设置
};

} // namespace gnss 
} // namespace front_end

#endif // __GNSS_PARAM_H__