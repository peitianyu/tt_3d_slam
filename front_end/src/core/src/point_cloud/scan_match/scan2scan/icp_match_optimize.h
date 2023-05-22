#ifndef __ICP_MATCH_OPTIMIZE_H__
#define __ICP_MATCH_OPTIMIZE_H__


#include "../scan_match_base.h"
#include "3dr_party/nanoflann.hpp"
#include "common/tt_math.h"
#include <Eigen/Dense>

namespace front_end{
namespace point_cloud{

    using icp_kdtree = nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd, 3, nanoflann::metric_L2_Simple>;

    class IcpMatchOptimize : public ScanMatchBase
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

        IcpMatchOptimize(const Option &option = Option()) : m_option(option) {}

        void Match(const std::vector<Point3D> &ref_ps, const std::vector<Point3D> &cur_ps, const Pose3D &prior_pose)
        {
            if (!CheckPointCloud(ref_ps, cur_ps))
                return;

            Reset(ref_ps, prior_pose);
            icp_kdtree ref_kdtree(3, m_ref_mat, 10);

            for (uint i = 0; i < m_option.max_iter; i++)
            {
                if (EstimateTransformationOnce(ref_kdtree, cur_ps))
                    return;
            }
        }

    private:
        void Reset(const std::vector<Point3D> &ref_ps, const Pose3D &prior_pose)
        {
            ResetResult(prior_pose);

            m_ref_mat = Eigen::MatrixXd(ref_ps.size(), 3);
            for (uint i = 0; i < ref_ps.size(); i++)
                m_ref_mat.row(i) = ref_ps[i];
        }

        bool CheckPointCloud(const std::vector<Point3D> &ref_ps, const std::vector<Point3D> &cur_ps)
        {
            if (ref_ps.size() < 100 || cur_ps.size() < 100)
            {
                std::cout << "point cloud size is too small" << std::endl;
                return false;
            }
            return true;
        }

        bool EstimateTransformationOnce(const icp_kdtree &ref_kdtree, const std::vector<Point3D> &cur_ps)
        {
            std::vector<Point3D> world_ps = TransformPointCloud(cur_ps, m_result.robot_pose);

            Eigen::Matrix3d d_R = m_result.robot_pose.Rot().ToMatrix();
            uint good_point_cnt = 0;
            float score = 0.0;

            Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
            Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
            for (uint i = 0; i < world_ps.size(); i++)
            {
                Point3D nearest_p;
                if (!FindNearestPoint(ref_kdtree, world_ps[i], nearest_p))
                    continue;

                Eigen::Vector3d error = world_ps[i] - nearest_p;
                Eigen::Matrix<double, 3, 6> jacobian = Eigen::Matrix<double, 3, 6>::Zero();
                jacobian.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
                jacobian.block<3, 3>(0, 3) = -d_R * common::Hat(cur_ps[i]);

                H += jacobian.transpose() * jacobian;
                b += jacobian.transpose() * error;

                ++good_point_cnt;
                score += error.norm();
            }

            if (H.determinant() == 0)
                return false;

            Eigen::Matrix<double, 6, 1> delta_x = -H.inverse() * b;

            m_result.error = delta_x.norm();
            m_result.score = score / good_point_cnt;
            m_result.robot_pose = m_result.robot_pose.TransformAdd(Pose3D(delta_x));
            m_result.is_converged = m_result.error < m_option.max_error;

            return m_result.is_converged;
        }

        bool FindNearestPoint(const icp_kdtree &ref_kdtree, const Point3D &p, Point3D &nearest_p)
        {
            size_t ret_index;
            double out_dist_sqr;
            nanoflann::KNNResultSet<double> result_set(1);
            result_set.init(&ret_index, &out_dist_sqr);
            ref_kdtree.index_->findNeighbors(result_set, &p[0]);

            if (out_dist_sqr > m_option.search_dist)
                return false;

            nearest_p = m_ref_mat.row(ret_index);
            return true;
        }

    private:
        Option m_option;
        Eigen::MatrixXd m_ref_mat;
};




} // namespace point_cloud
} // namespace front_end


#endif // __ICP_MATCH_OPTIMIZE_H__