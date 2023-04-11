#ifndef __TYPES_POINT3D_H__
#define __TYPES_POINT3D_H__
#include <Eigen/Core>
#include "time.h"

namespace types {

using Point3D = Eigen::Vector3d;

struct TimedPoint3D
{
    Time time_stamp;
    Point3D point;

    TimedPoint3D(const Time& time = 0.0, const Point3D& point = Point3D::Zero())
        : time_stamp(time), point(point)
    {}
};


} // namespace types


#endif // __TYPES_POINT3D__