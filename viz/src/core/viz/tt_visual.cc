#include "tt_visual.h"

namespace viz{

Visual::Visual()
{
    #ifdef USE_PCL_VIZ
    m_window.setBackgroundColor(255, 255, 255);
    #else
    m_window.setBackgroundColor(cv::viz::Color::white());
    #endif // USE_PCL_VIZ
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
    #ifdef USE_PCL_VIZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(uint i = 0; i < points.size(); i++)
        point_cloud->push_back(Point3D2PointXYZ(points[i]));
    ShowWidget(name, point_cloud, color);
    SetPointSize(name, VIZ_POINT_SIZE);
    #else
    cv::Mat points_mat(points.size(), 1, CV_32FC3);
    for(uint i = 0; i < points.size(); i++)
        points_mat.at<cv::Vec3f>(i) = cv::Vec3f(points[i].x(), points[i].y(), points[i].z());
    cv::viz::WCloud cloud_widget(points_mat, cv::viz::Color(color.r, color.g, color.b));
    m_window.showWidget(name, cloud_widget);
    #endif // USE_PCL_VIZ
}

void Visual::ShowPointCloud(types::Pose3D predict, std::vector<types::Point3D> points, Color color)
{
    #ifdef USE_PCL_VIZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(uint i = 0; i < points.size(); i++)
        point_cloud->push_back(Point3D2PointXYZ(predict.TransformAdd(points[i])));

    ShowWidget("point_cloud1", point_cloud, color);
    SetPointSize("point_cloud1", VIZ_POINT_SIZE);
    #else
    cv::Mat points_mat(points.size(), 1, CV_32FC3);
    for(uint i = 0; i < points.size(); i++)
        points_mat.at<cv::Vec3f>(i) = cv::Vec3f(points[i].x(), points[i].y(), points[i].z());
    cv::viz::WCloud cloud_widget(points_mat, cv::viz::Color(color.r, color.g, color.b));
    m_window.showWidget("point_cloud1", cloud_widget);
    #endif // USE_PCL_VIZ
}

void Visual::ShowGridMap(std::shared_ptr<grid_map::GridMapBase> grid_map)
{
    #ifdef USE_PCL_VIZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const std::pair<grid_map::Index3D, double> &kv : grid_map->GetData())
        point_cloud->push_back(Point3D2PointXYZ(kv.first.Index2Point()));

    ShowWidget("grid_map", point_cloud, COLOR_GRAY);
    SetPointSize("grid_map", VIZ_POINT_SIZE);
    #else
    cv::Mat points_mat(grid_map->GetData().size(), 1, CV_32FC3);
    for (const std::pair<grid_map::Index3D, float> &kv : grid_map->GetData())
        points_mat.push_back(cv::Vec3f(kv.first.x(), kv.first.y(), kv.first.z()));
    cv::viz::WCloud cloud_widget(points_mat, cv::viz::Color(COLOR_GRAY.r, COLOR_GRAY.g, COLOR_GRAY.b));
    m_window.showWidget("grid_map", cloud_widget);
    #endif // USE_PCL_VIZ
}

void Visual::ShowTrajectory(const std::vector<types::TimedPose3D>& timed_poses)
{
    #ifdef USE_PCL_VIZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for(const types::TimedPose3D& timed_pose : timed_poses)
        point_cloud->push_back(Point3D2PointXYZ(timed_pose.pose.Point()));
    
    ShowWidget("trajectory", point_cloud, COLOR_BLUE);
    SetPointSize("trajectory", 50);
    #else
    cv::Mat points_mat(timed_poses.size(), 1, CV_32FC3);
    for(uint i = 0; i < timed_poses.size(); i++)
        points_mat.at<cv::Vec3f>(i) = cv::Vec3f(timed_poses[i].pose.Point().x(), timed_poses[i].pose.Point().y(), timed_poses[i].pose.Point().z());
    cv::viz::WCloud cloud_widget(points_mat, cv::viz::Color(COLOR_BLUE.r, COLOR_BLUE.g, COLOR_BLUE.b));
    m_window.showWidget("trajectory", cloud_widget);
    #endif // USE_PCL_VIZ
}

#ifdef USE_PCL_VIZ
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
#endif // USE_PCL_VIZ

void Visual::SetPointSize(const std::string& name, double size)
{
    std::lock_guard<std::mutex> lock(m_data_mutex);
    #ifdef USE_PCL_VIZ
    m_window.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, name);
    #endif // USE_PCL_VIZ
}

} // namespace viz
