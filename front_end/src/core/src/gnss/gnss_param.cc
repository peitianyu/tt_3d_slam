#include "gnss_param.h"

namespace front_end{
namespace gnss{

types::Pose3D GnssParam::GetAntennaPose() const { return types::Pose3D(Eigen::Vector3d(antenna_offset(0), antenna_offset(1), 0), types::Rot3D(0, 0, antenna_offset(2))); }

Eigen::Matrix3d GnssParam::GetTransCov() const
{
    double trans_noise2 = trans_noise * trans_noise;
    return Eigen::Matrix3d::Identity() * trans_noise2;
}

Eigen::Matrix3d GnssParam::GetRotCov() const
{
    double rot_noise2 = rot_noise * rot_noise;
    return Eigen::Matrix3d::Identity() * rot_noise2;
}

GnssParam::GnssParam(bool u, double t_n, double r_n, Eigen::Vector3d a, GpsStatusType s) 
            : using_heading(u), trans_noise(t_n), rot_noise(r_n), antenna_offset(a), status(s) {}

GnssParam::GnssParam() 
            : GnssParam(true, 0.1, 0.1, Eigen::Vector3d::Zero(), GpsStatusType::GNSS_OTHER) {}


} // namespace gnss
} // namespace front_end