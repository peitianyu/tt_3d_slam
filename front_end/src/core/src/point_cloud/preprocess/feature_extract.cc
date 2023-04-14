#include "feature_extract.h"
#include "../filter/voxel_filter.h"
#include "viz/tt_visual.h"

namespace front_end{
namespace point_cloud{

FeatureExtract::FeatureExtract(const Param& param) : m_param(param) {}

FeatureExtract::FeatureExtract(const PointCloudParam& p_c_p, const uint& s_s, const double& d_r)
    : m_param(Param(s_s, d_r, p_c_p)) {}

void FeatureExtract::Extract(const front_end::point_cloud::TimedPointCloudPCD& timed_point_cloud)
{
    if(!PreprocessPointCloud(timed_point_cloud.point_cloud))
        return;

    CalculateSmoothness();

    MarkOccludedParallelPoints();

    ExtractCornerSurfacePoints();
}

const std::unordered_map<std::string, std::vector<types::Point3D>>& FeatureExtract::GetFeature() const { return m_point_cloud; }

bool FeatureExtract::PreprocessPointCloud(const front_end::point_cloud::PointCloudPCD& point_cloud)
{
    constexpr uint min_point_size = 100;
    if (point_cloud.GetPointCloud().empty() || point_cloud.GetPoints().size() < min_point_size)
        return false;

    m_cloud_points = point_cloud.GetPointCloud();
    for (auto& iter : m_cloud_points)
        std::sort(iter.second.begin(), iter.second.end(), [](const CloudPointType& a, const CloudPointType& b){ return a.index < b.index; });
    
    return true;
}

void FeatureExtract::CalculateSmoothness()
{
    m_cloud_smoothness.clear();

    for(const auto& iter: m_cloud_points)
    {
        uint line_num = iter.first;
        if(iter.second.size() < m_param.smoothness_size + 1)
            continue;

        m_cloud_smoothness[line_num].resize(iter.second.size());
        for (uint i = 0; i < m_param.smoothness_size; i++)
            m_cloud_smoothness[line_num][i] = CloudSmoothness(false, 0.0f, CloudLable::IDLE, i);

        for (uint i = m_param.smoothness_size; i < iter.second.size() - m_param.smoothness_size; i++)
        {
            double diff_range = 0.0f;
            for (uint j = 1; j <= m_param.smoothness_size; j++){
                diff_range += iter.second[i - j].Range();
                diff_range += iter.second[i + j].Range();
            }
            diff_range -= iter.second[i].Range() * (2 * m_param.smoothness_size);

            m_cloud_smoothness[line_num][i] = CloudSmoothness(false, diff_range * diff_range, CloudLable::IDLE, i);
        }
    }
}

void FeatureExtract::MarkOccludedParallelPoints()
{
    for(auto& iter: m_cloud_smoothness)
    {
        uint line_num = iter.first;
        if(iter.second.size() < m_param.smoothness_size + 1)
            continue;

        for (uint i = m_param.smoothness_size + 1; i < m_cloud_points[line_num].size() - m_param.smoothness_size - 1; i++)
        {
            double depth1 = m_cloud_points[line_num][i].Range();
            double depth2 = m_cloud_points[line_num][i+1].Range();
            double depth3 = m_cloud_points[line_num][i-1].Range();
            uint column_diff = std::abs(int(m_cloud_points[line_num][i+1].index - m_cloud_points[line_num][i].index));
            
            constexpr uint column_diff_max = 10;
            if (column_diff < column_diff_max)
            {
                constexpr double occluded_thr = 0.3;
                if(depth1 - depth2 > occluded_thr){
                    for(uint j = 0; j < m_param.smoothness_size + 1; j++)
                        m_cloud_smoothness[line_num][i - j].picked = true;
                }else if(depth2 - depth1 > occluded_thr){
                    for(uint j = 1; j < m_param.smoothness_size + 2; j++)
                        m_cloud_smoothness[line_num][i + j].picked = true;
                }
            }  

            double diff1 = std::abs(double(depth1 - depth2));
            double diff2 = std::abs(double(depth1 - depth3));

            constexpr double parallel_thr = 0.02;
            if (diff1 > parallel_thr * depth1 && diff2 > parallel_thr * depth1)
                m_cloud_smoothness[line_num][i].picked = true;
        }
    }
}

void FeatureExtract::ExtractCornerSurfacePoints()
{
    for(auto& iter: m_cloud_smoothness)
    {
        uint line_num = iter.first;
        if(iter.second.size() < m_param.smoothness_size + 1)
            continue;
        
        constexpr uint segment_num = 6;
        for(uint i = 0; i < segment_num; i++)
        {
            uint start_index = i * m_cloud_smoothness[line_num].size() / segment_num;
            uint end_index = (i + 1) * m_cloud_smoothness[line_num].size() / segment_num - 1;

            std::sort(m_cloud_smoothness[line_num].begin() + start_index, m_cloud_smoothness[line_num].begin() + end_index, 
                [](const CloudSmoothness& a, const CloudSmoothness& b) { return a.curvature > b.curvature; });

            constexpr uint max_corner_num = 20;

            // 先提取角点, 按照曲率从大到小排序
            uint corner_cnt = 0;
            for(uint j = start_index; j < end_index; j++)
            {
                if (m_cloud_smoothness[line_num][j].picked)
                    continue;

                constexpr double edge_thr = 1.0;
                if(m_cloud_smoothness[line_num][j].curvature > edge_thr){
                    m_cloud_smoothness[line_num][j].picked = true;
                    m_cloud_smoothness[line_num][j].label = CloudLable::CORNER;
                    m_point_cloud["corner"].push_back(m_cloud_points[line_num][j].timed_point.point);
                    
                    if(!PickedCloudNeighbor(line_num, j))
                        continue;
                    
                    corner_cnt++;
                    if(corner_cnt > max_corner_num)
                        break;
                }
            }

            // 再提取平面点, 按照曲率从小到大排序
            for(uint j = end_index; j > start_index; j--)
            {
                if(m_cloud_smoothness[line_num][j].picked)
                    continue;

                constexpr double surface_thr = 0.1;
                if(m_cloud_smoothness[line_num][j].curvature < surface_thr){
                    m_cloud_smoothness[line_num][j].picked = true;
                    m_cloud_smoothness[line_num][j].label = CloudLable::SURFACE;
                    m_point_cloud["surface"].push_back(m_cloud_points[line_num][j].timed_point.point);

                    if (!PickedCloudNeighbor(line_num, j))
                        continue;
                }
            }
        }
    }

    filter::VoxelFilter(m_point_cloud["surface"], m_param.downsample_resolution);
}

bool FeatureExtract::PickedCloudNeighbor(const uint& line_num, const uint& index)
{
    if(index < m_param.smoothness_size || index > m_cloud_points[line_num].size() - m_param.smoothness_size - 1)
        return false;

    for (uint k = 1; k <= m_param.smoothness_size; k++)
    {
        uint column_diff0 = std::abs(int(m_cloud_points[line_num][index + k].index) - int(m_cloud_points[line_num][index + k - 1].index));
        uint column_diff1 = std::abs(int(m_cloud_points[line_num][index - k].index) - int(m_cloud_points[line_num][index - k + 1].index));

        if (column_diff0 > 2 * m_param.smoothness_size || column_diff1 > 2 * m_param.smoothness_size)
            break;

        m_cloud_smoothness[line_num][index + k].picked = true;
        m_cloud_smoothness[line_num][index - k].picked = true;
    }

    return true;
}

} // namespace point_cloud
} // namespace front_end

