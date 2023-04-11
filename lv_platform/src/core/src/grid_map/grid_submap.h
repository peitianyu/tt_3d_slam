#ifndef __GRID_SUBMAP_H__
#define __GRID_SUBMAP_H__

#include "grid_map_base.h"
#include "types/timed_pose3d.h"
#include <memory>


namespace grid_map{

// TODO: 初步设想是加速匹配运算过程
struct GridSubMap
{
    uint id;
    types::TimedPose3D map_pose;
    std::shared_ptr<GridMapBase> grid_map;
};

} // namespace grid_map


#endif // __GRID_SUBMAP_H__
