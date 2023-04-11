#include"gnss.h"

namespace front_end{
namespace gnss{

Gnss::Gnss() : lv_math::types::TimedPose3D(), valid(false), north(true), zone(0), param() {}

Gnss::Gnss(const Gnss& other) : TimedPose3D(other), valid(other.valid), north(other.north), zone(other.zone), param(other.param) {}

// 根据不同厂家的数据格式，需要自己实现
Gnss::Gnss(const lv_math::types::Time &t, const bool &v, const Eigen::Vector3d &lat_lon_alt, const double &heading, const GnssParam &p)
{
    time_stamp = t;
    valid = v;
    param = p;
    pose = lv_math::types::Pose3D(LatLonAlt2XYZ(lat_lon_alt), HeadingToRot(heading)) 
                                .TransformAdd(param.GetAntennaPose().TransformFrom(lv_math::types::Pose3D()));
}

Eigen::Vector3d Gnss::LatLonAlt2XYZ(const Eigen::Vector3d &lat_lon_alt)
{
    Eigen::Vector3d result(0.0, 0.0, lat_lon_alt(2));
    UTM::LLtoUTM(lat_lon_alt(0), lat_lon_alt(1), result(0), result(1), zone, north);
    return result;
}

lv_math::types::Rot3D Gnss::HeadingToRot(const double &heading) { return lv_math::types::Rot3D(0, 0, (90 - heading)/180.0*M_PI);}

} // namespace gnss
} // namespace front_end