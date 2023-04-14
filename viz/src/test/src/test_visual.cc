#include<iostream>
#include"viz/tt_visual.h"
#include"common/tt_log.h"

using namespace viz;
using namespace types;
using namespace grid_map;

std::vector<Point3D> BuildRandomPoints()
{
    std::vector<Point3D> points;
    for(int i = 0; i < 100; ++i)
    {
        Point3D point;
        point.x() = double(rand() % 1000)/100.0;
        point.y() = double(rand() % 1000)/100.0;
        point.z() = double(rand() % 1000)/100.0;
        // std::cout << point.transpose() << std::endl;
        points.push_back(point);
    }

    return points;
}

void TestVisual()
{
    std::vector<Point3D> points = BuildRandomPoints();
    Pose3D pose(Point3D(1.0, 1.0, 1.0), Rot3D(Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5)));

    // 测试显示点云
    Visual::GetInstance()->ShowPointCloud(points);

    // 测试显示位姿点云
    Visual::GetInstance()->ShowPointCloud(pose, points);

    // 测试显示grid_map
    std::shared_ptr<GridMapBase> grid_map(new GridMapBase());
    grid_map->UpdateByScan(pose, points);
    Visual::GetInstance()->ShowGridMap(grid_map);

    // 显示
    Visual::GetInstance()->Show();
}

int main()
{
    TestVisual();
    return 0;
}