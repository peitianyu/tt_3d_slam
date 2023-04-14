#ifndef __FRONT_END_FEATURE_EXTRACT_H__
#define __FRONT_END_FEATURE_EXTRACT_H__

#include "../types/point_cloud.h"
#include "../types/cloud_smoothness.h"
#include "../types/point_cloud_param.h"
#include "types/point3d.h"
#include <unordered_map>

namespace front_end{
namespace point_cloud{

class FeatureExtract
{
public:
    struct Param
    {
        uint smoothness_size;
        double downsample_resolution;

        PointCloudParam point_cloud_param;

        Param(const uint& s_s = 5, const double& d_r = 1.0, const PointCloudParam& p_c_p = PointCloudParam())
            : smoothness_size(s_s), downsample_resolution(d_r), point_cloud_param(p_c_p)
        {}
    };

    FeatureExtract(const Param& param = Param());

    FeatureExtract(const PointCloudParam &p_c_p, const uint &s_s = 5, const double &d_r = 1.0);

    void Extract(const front_end::point_cloud::TimedPointCloudPCD& timed_point_cloud);

    const std::unordered_map<std::string, std::vector<types::Point3D>>& GetFeature() const;
private:
    bool PreprocessPointCloud(const front_end::point_cloud::PointCloudPCD& point_cloud);

    void CalculateSmoothness();

    void MarkOccludedParallelPoints();

    void ExtractCornerSurfacePoints();

    bool PickedCloudNeighbor(const uint& line_num, const uint& index);
private:
    Param m_param;
    std::unordered_map<std::string, std::vector<types::Point3D>> m_point_cloud;
    std::unordered_map<uint, std::vector<CloudSmoothness>> m_cloud_smoothness; 
    std::map<uint, std::vector<CloudPointType>> m_cloud_points;
};

} // namespace point_cloud
} // namespace front_end

#endif // __FRONT_END_FEATURE_EXTRACT_H__