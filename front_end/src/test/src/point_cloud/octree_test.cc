#include "common/tt_test.h"
#include "point_cloud/octree_map/octree.h"
#include "viz/tt_visual.h"
#include "load_pcd.h"

// JUST_RUN_TEST(octree, test)
TEST(octree, test)
{
    front_end::point_cloud::OctreeMap octree_map;
    std::vector<types::Point3D> ref_ps = load_pcd("/mnt/d/file_ws/Learning/slam/tt_3d_slam/front_end/src/test/data/rabbit3.pcd");
    octree_map.InsertPointCloud(ref_ps);

    types::Point3D point(-3.4838514, 6.3016939, -3.7409768);
    types::Point3D nearest_point = octree_map.GetNearestPoint(point);
    std::cout << "nearest_point: " << nearest_point.transpose() << std::endl;

    std::vector<types::Point3D> points;
    octree_map.GetNearestPoints(point, 0.5, points);
    std::cout << "points: " << points.size() << std::endl;

    std::cout << "is_valid: " << octree_map.IsValid(point) << std::endl;
}