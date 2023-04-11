#ifndef __TYPES_CLOUD_SMOOTHNESS_H__
#define __TYPES_CLOUD_SMOOTHNESS_H__

#include<iostream>

namespace front_end{
namespace point_cloud{

enum CloudLable
{
    IDLE,
    CORNER,
    SURFACE,
    GROUND
};

struct CloudSmoothness
{   
    bool picked;
    double curvature;
    CloudLable label;
    uint index;

    CloudSmoothness(bool p, double c, CloudLable l, uint i)
        : picked(p), curvature(c), label(l), index(i)
    {}

    CloudSmoothness()
        : CloudSmoothness(false, 0.0f, CloudLable::IDLE, 0)
    {}
};

} // namespace point_cloud
} // namespace front_end

#endif // __TYPES_CLOUD_SMOOTHNESS_H__