#ifndef __NDT_MATHCH_H__
#define __NDT_MATHCH_H__

#include "../scan_match_base.h"
#include "3dr_party/nanoflann.hpp"
#include "common/tt_math.h"
#include <Eigen/Dense>
#include <cassert>
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
        uint voxel_size;
        double resolution;

        constexpr uint max_grid_size() const
        {
            return voxel_size * voxel_size * voxel_size;
        }

        Option(uint max_iter = 100, double max_error = 1e-5, double search_dist = 2.0, double voxel_size = 200, double resolution = 2.0)
            : max_iter(max_iter), max_error(max_error), search_dist(search_dist), voxel_size(voxel_size), resolution(resolution) {}
    };

    struct GridCell
    {
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();;
        Eigen::Matrix3d covarince = Eigen::Matrix3d::Zero();
        std::vector<Point3D> points;

        double GetProbability(const Point3D &p)
        {
            Eigen::Matrix3d inv_cov = covarince.inverse();
            double gauss_d2 = (p - mean).transpose() * inv_cov * (p - mean);
            return exp(-0.5 * gauss_d2);
        }
    };

    NdtMatch(const Option &option = Option()) : m_option(option) {}

    void Match(const std::vector<Point3D> &ref_ps, const std::vector<Point3D> &cur_ps, const Pose3D &prior_pose)
    {
        if (!CheckPointCloud(ref_ps, cur_ps))
            return;

        Reset(ref_ps, prior_pose);

        for (uint i = 0; i < m_option.max_iter; i++){
            if (EstimateTransformationOnce(cur_ps))
                return;
        }
    }

    std::vector<Point3D> GetGridPoints() const 
    {
        std::vector<Point3D> grid_points;
        for (const auto &cell : m_grid_cells){
            if(cell.points.empty())
                continue;
            
            grid_points.push_back(cell.mean);
        }
        return grid_points;
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
        m_grid_cells.clear();
        m_grid_cells.resize(m_option.max_grid_size());

        for (Point3D point : ref_ps){
            uint index = PointToGrid(point);
            m_grid_cells[index].mean += point;
            m_grid_cells[index].points.push_back(point);
        }

        uint size = 0;
        for (size_t i = 0; i < m_grid_cells.size(); i++){
            if (m_grid_cells[i].points.size() >= 3){
                m_grid_cells[i].mean = m_grid_cells[i].mean / static_cast<double>(m_grid_cells[i].points.size());

                for (Point3D point : m_grid_cells[i].points)
                    m_grid_cells[i].covarince += (point - m_grid_cells[i].mean) * (point - m_grid_cells[i].mean).transpose();
                m_grid_cells[i].covarince /= static_cast<double>(m_grid_cells[i].points.size()-1);

                ++size;
            }
            else{
                m_grid_cells[i].mean = m_grid_cells[i].mean / static_cast<double>(m_grid_cells[i].points.size());
                constexpr double double_max = 0.25;
                m_grid_cells[i].covarince << double_max, 0, 0, 0, double_max, 0, 0, 0, double_max;
            }
        }

        std::cout << "grid cell size: " << size << std::endl;
    }

    void Reset(const std::vector<Point3D> &ref_ps, const Pose3D &prior_pose)
    {
        ResetResult(prior_pose);

        BuildVoxelGridMap(ref_ps);
    }
    
    bool EstimateTransformationOnce(const std::vector<Point3D> &cur_ps)
    {
        std::vector<Point3D> world_ps = TransformPointCloud(cur_ps, m_result.robot_pose);
        Eigen::Matrix3d d_R = m_result.robot_pose.Rot().ToMatrix();
        uint good_point_cnt = 0;
        float score = 0.0;

        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
        for (uint i = 0; i < world_ps.size(); i++)
        {
            uint index = PointToGrid(world_ps[i]);
            if(index >= m_option.max_grid_size() || m_grid_cells[index].points.size() < 5)
                continue;

            Eigen::Vector3d error = world_ps[i] - m_grid_cells[index].mean;
            Eigen::Matrix3d inv_cov = m_grid_cells[index].covarince.inverse();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(inv_cov);
            Eigen::Matrix3d sqrt_inv_cov = eig.operatorSqrt();

            Eigen::Matrix<double, 3, 6> jacobian = Eigen::Matrix<double, 3, 6>::Zero();
            jacobian.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            jacobian.block<3, 3>(0, 3) = -d_R * common::Hat(cur_ps[i]);

            // H += jacobian.transpose() * jacobian;
            // b += jacobian.transpose() * error;

            H += jacobian.transpose() * inv_cov * jacobian;
            b += jacobian.transpose() * sqrt_inv_cov * error;

            // if(i%100 == 0){
            //     // std::cout << "error: " << error.transpose() << std::endl;
            //     std::cout << "cov: \n" << m_grid_cells[index].covarince << std::endl;
            //     // std::cout << "inv_cov: \n" << inv_cov << std::endl;
            //     // std::cout << "sqrt_inv_cov: \n" << sqrt_inv_cov << std::endl;
            //     // std::cout << "jacobian: \n" << jacobian << std::endl;
            //     // std::cout << "H: \n" << H << std::endl;
            //     // std::cout << "b: \n"
            //     //           << b.transpose() << std::endl;
            // }

            ++good_point_cnt;
            score += error.norm();
        }

        if (H.determinant() == 0)
            return false;

        Eigen::Matrix<double, 6, 1> delta_x = -H.inverse() * b;

        // std::cout << "H: \n" << H << std::endl;
        // std::cout << "b: \n" << b << std::endl;

        // std::cout << "good_point_cnt: " << good_point_cnt << std::endl;

        m_result.error = delta_x.norm();
        m_result.score = score / good_point_cnt;
        m_result.robot_pose = m_result.robot_pose.TransformAdd(Pose3D(delta_x));
        m_result.is_converged = m_result.error < m_option.max_error;

        std::cout << "score: " << m_result.score << " error: " << m_result.error << " good_point_cnt: " << good_point_cnt << std::endl;


        return m_result.is_converged;
    }

    const uint PointToGrid( const Point3D &point ) const
    {
        int x = static_cast<int>(point(0) / m_option.resolution) + m_option.voxel_size/2;
        int y = static_cast<int>(point(1) / m_option.resolution) + m_option.voxel_size/2;
        int z = static_cast<int>(point(2) / m_option.resolution) + m_option.voxel_size/2;

        assert(x >= 0 && x < m_option.voxel_size);
        assert(y >= 0 && y < m_option.voxel_size);
        assert(z >= 0 && z < m_option.voxel_size);

        return x + y * m_option.voxel_size + z * m_option.voxel_size * m_option.voxel_size;
    }
private:
    Option m_option;
    std::vector<GridCell> m_grid_cells;
};




} // namespace point_cloud
} // namespace front_end




#endif // __NDT_MATHCH_H__