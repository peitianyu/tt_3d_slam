#include"voxel_filter.h"

namespace front_end{
namespace point_cloud{
namespace filter{

void VoxelFilter(std::vector<lv_math::types::Point3D>& point_cloud, const double& resolution, const double& range_max)
{
    int index = std::numeric_limits<int>::max();
    std::map<int, std::vector<lv_math::types::Point3D>> points_map;
    int grid_size = int(range_max / resolution);
    for (lv_math::types::Point3D point : point_cloud)
    {
        index = int(point.x() / resolution) + int(point.y() / resolution) * grid_size + int(point.z() / resolution) * grid_size * grid_size;
        points_map[index].push_back(point);
    }

    point_cloud.clear();
    for (auto point_pair : points_map)
    {
        lv_math::types::Point3D sum_point = lv_math::types::Point3D::Zero();
        for (lv_math::types::Point3D &p : point_pair.second)
            sum_point += p;

        point_cloud.push_back(sum_point / static_cast<double>(point_pair.second.size()));
    }
}

} // namespace filter
} // namespace point_cloud
} // namespace front_end

