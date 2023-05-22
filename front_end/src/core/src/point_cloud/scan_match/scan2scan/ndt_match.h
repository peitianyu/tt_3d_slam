#ifndef __NDT_MATHCH_H__
#define __NDT_MATHCH_H__

#include "../scan_match_base.h"
#include "3dr_party/nanoflann.hpp"
#include "common/tt_math.h"
#include <Eigen/Dense>

namespace front_end{
namespace point_cloud{

class NdtMatch : public ScanMatchBase
{
public:
    struct Option
    {
        uint max_iter;
        double max_error;
        double search_dist;

        Option(uint max_iter = 100, double max_error = 1e-5, double search_dist = 2.0)
            : max_iter(max_iter), max_error(max_error), search_dist(search_dist) {}
    };

    NdtMatch(const Option &option = Option()) : m_option(option) {}

    void Match(const std::vector<Point3D> &ref_ps, const std::vector<Point3D> &cur_ps, const Pose3D &prior_pose)
    {
        if (!CheckPointCloud(ref_ps, cur_ps))
            return;

        Reset(ref_ps, prior_pose);

        BuildVoxelGridMap(ref_ps);

        for (uint i = 0; i < m_option.max_iter; i++)
        {
            
        }
    }

private:
    bool CheckPointCloud(const std::vector<Point3D> &ref_ps, const std::vector<Point3D> &cur_ps)
    {
        if (ref_ps.size() < 100 || cur_ps.size() < 100){
            std::cout << "point cloud size is too small" << std::endl;
            return false;
        }
        return true;
    }

    void BuildVoxelGridMap(const std::vector<Point3D> &ref_ps)
    {
        // m_voxel_grid_map.clear();
        // for (uint i = 0; i < ref_ps.size(); i++)
        // {
        //     Point3D p = ref_ps[i];
        //     int x = (int)(p.x / m_voxel_size);
        //     int y = (int)(p.y / m_voxel_size);
        //     int z = (int)(p.z / m_voxel_size);
        //     m_voxel_grid_map[std::make_tuple(x, y, z)].push_back(p);
        // }
    }

    void Reset(const std::vector<Point3D> &ref_ps, const Pose3D &prior_pose)
    {
        ResetResult(prior_pose);

        BuildVoxelGridMap(ref_ps);
    }

    
    bool EstimateTransformationOnce(const std::vector<Point3D> &cur_ps)
    {
        
    }

private:
    Option m_option;
};




} // namespace point_cloud
} // namespace front_end




#endif // __NDT_MATHCH_H__