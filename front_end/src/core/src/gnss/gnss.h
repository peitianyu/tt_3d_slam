#ifndef __TYPES_SENSOR_GNSS_H__
#define __TYPES_SENSOR_GNSS_H__

#include "types/pose3d.h"
#include "types/time.h"
#include "utm.h"
#include "gnss_param.h"

namespace front_end{
namespace gnss{


// 这里主要设计得是RTK数据结构
struct Gnss : public lv_math::types::TimedPose3D
{
    bool valid;
    bool north;
    int zone;
    GnssParam param;

    Gnss();
    Gnss(const Gnss& other);
    // 根据不同厂家的数据格式，需要自己实现
    Gnss(const lv_math::types::Time &t, const bool& v, const Eigen::Vector3d &lat_lon_alt, 
        const double &heading = 0.0, const GnssParam &p = GnssParam());
private:
    Eigen::Vector3d LatLonAlt2XYZ(const Eigen::Vector3d &lat_lon_alt);
    lv_math::types::Rot3D HeadingToRot(const double &heading);
};

} // namespace gnss
} // namespace front_end

#endif /* __TYPES_SENSOR_GNSS_H__ */