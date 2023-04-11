#ifndef __POINT_CLOUD_VOXEL_FILTER_H__
#define __POINT_CLOUD_VOXEL_FILTER_H__

#include"types/point3d.h"
#include<vector>
#include<map>

namespace front_end{
namespace point_cloud{
namespace filter{

void VoxelFilter(std::vector<lv_math::types::Point3D>& point_cloud, const double& resolution, const double& range_max = 100.0);

} // namespace filter
} // namespace point_cloud
} // namespace front_end

#endif // __POINT_CLOUD_VOXEL_FILTER_H__