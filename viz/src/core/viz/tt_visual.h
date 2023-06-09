#ifndef __VIZ_VISUAL_H__
#define __VIZ_VISUAL_H__

#ifdef USE_PCL_VIZ
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#else
#include <opencv2/viz.hpp>
#endif // USE_PCL_VIZ

#include <mutex>
#include <memory>
#include <thread>

#include "common/tt_singleton.h"
#include "grid_map/grid_map_base.h"
#include "types/pose3d.h"
#include "types/point3d.h"
#include "tt_color.h"

namespace viz{

#define VIZ_POINT_SIZE 6
class Visual
{
public:
    static Visual* GetInstance()
    {
        #ifdef USE_PCL_VIZ
        std::cout << "USE_PCL_VIZ is not defined, use PCLVisualizer" << std::endl;
        #endif // USE_PCL_VIZ
        static Singleton<Visual> s_instance;
        return s_instance.Get();
    }

    Visual();

    void ShowOnce();

    void Show();

    void ShowPointCloud(std::vector<types::Point3D> points, Color color = COLOR_BLUE);

    void ShowPointCloud(std::string name, std::vector<types::Point3D> points, Color color = COLOR_BLUE);

    void ShowPointCloud(types::Pose3D predict, std::vector<types::Point3D> points, Color color = COLOR_RED);

    void ShowGridMap(std::shared_ptr<grid_map::GridMapBase> grid_map);

    void ShowTrajectory(const std::vector<types::TimedPose3D>& timed_poses);
private:
    #ifdef USE_PCL_VIZ
    void ShowWidget(const std::string& name, const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, Color color);
    pcl::PointXYZ Point3D2PointXYZ(const types::Point3D& p);
    #endif // USE_PCL_VIZ

    void SetPointSize(const std::string& name, double size);

private:
    mutable std::mutex m_data_mutex;

    #ifdef USE_PCL_VIZ
    pcl::visualization::PCLVisualizer m_window;
    #else
    cv::viz::Viz3d m_window;
    #endif // USE_PCL_VIZ
};

} // namespace viz

#endif // __VIZ_VISUAL_H__