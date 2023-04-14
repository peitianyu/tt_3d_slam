#include "tt_visual.h"

namespace viz{


Visual::Visual()
{
    m_window.setBackgroundColor(255, 255, 255);
}

void Visual::Show()
{
    while(!m_window.wasStopped())
    {
        m_window.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void Visual::ShowOnce()
{
    m_window.spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void Visual::ShowPointCloud(std::vector<types::Point3D> points, Color color)
{
    ShowPointCloud("point_cloud0", points, color);
}

void Visual::ShowPointCloud(std::string name, std::vector<types::Point3D> points, Color color)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(uint i = 0; i < points.size(); i++)
        point_cloud->push_back(Point3D2PointXYZ(points[i]));

    ShowWidget(name, point_cloud, color);
    SetPointSize(name, VIZ_POINT_SIZE);
}

void Visual::ShowPointCloud(types::Pose3D predict, std::vector<types::Point3D> points, Color color)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(uint i = 0; i < points.size(); i++)
        point_cloud->push_back(Point3D2PointXYZ(predict.TransformAdd(points[i])));

    ShowWidget("point_cloud1", point_cloud, color);
    SetPointSize("point_cloud1", VIZ_POINT_SIZE);
}

void Visual::ShowGridMap(std::shared_ptr<grid_map::GridMapBase> grid_map)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const std::pair<grid_map::Index3D, double> &kv : grid_map->GetData())
        point_cloud->push_back(Point3D2PointXYZ(kv.first.Index2Point()));

    ShowWidget("grid_map", point_cloud, COLOR_GRAY);
    SetPointSize("grid_map", VIZ_POINT_SIZE);
}

void Visual::ShowTrajectory(const std::vector<types::TimedPose3D>& timed_poses)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for(const types::TimedPose3D& timed_pose : timed_poses)
        point_cloud->push_back(Point3D2PointXYZ(timed_pose.pose.Point()));
    
    ShowWidget("trajectory", point_cloud, COLOR_BLUE);
    SetPointSize("trajectory", 50);
}

void Visual::ShowWidget(const std::string& name, const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, Color color)
{
    std::lock_guard<std::mutex> lock(m_data_mutex);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(point_cloud, color.r, color.g, color.b);
    m_window.removePointCloud(name);
    m_window.addPointCloud(point_cloud, color_handler, name);
}

pcl::PointXYZ Visual::Point3D2PointXYZ(const types::Point3D& p)
{
    return pcl::PointXYZ(p(0), p(1), p(2));
}

void Visual::SetPointSize(const std::string& name, double size)
{
    std::lock_guard<std::mutex> lock(m_data_mutex);
    m_window.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, name);
}


} // namespace viz
