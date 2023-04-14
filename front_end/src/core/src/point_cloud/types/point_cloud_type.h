#ifndef __POINT_CLOUD_TYPE_H__
#define __POINT_CLOUD_TYPE_H__

#include"types/point3d.h"

namespace front_end{
namespace point_cloud{

struct CloudPointType
{
    uint index;
    types::TimedPoint3D timed_point;

    double Range() const
    {
        return timed_point.point.norm();
    }

    CloudPointType(uint i, const types::TimedPoint3D& p)
        : index(i), timed_point(p)
    {}

    CloudPointType()
        : CloudPointType(0, types::TimedPoint3D())
    {}
};

} // namespace point_cloud
} // namespace front_end

#endif // __POINT_CLOUD_TYPE_H__