#ifndef __SCAN_CONTEXT_H__
#define __SCAN_CONTEXT_H__

#include<vector>
#include<cmath>
#include<memory>
#include<iostream>

#include "common/math.h"
#include "scan_context_kdtree.h"
#include "../loop_closure_base.h"
#include "viz/visual.h"
#include "3dr_party/nanoflann.hpp"

namespace front_end {
namespace loop_closure {


class ScanContext : public LoopClosureBase
{
public:
    struct Param
    {
        double max_range;
        double min_range;

        uint match_thr;
        uint num_exclude_recent;

        double search_radius;
        double contact_ratio_thr;

        Param(double mr = 60, double minr = 1.0, uint mt = 20, uint n_e_r = 15, double s_r = 1e4, double c_r_t = 1e2)
            : max_range(mr), min_range(minr), match_thr(mt), num_exclude_recent(n_e_r), search_radius(s_r), contact_ratio_thr(c_r_t) {}
    };

    ScanContext(const Param& param = Param());

    virtual bool Match(const std::vector<lv_math::types::Point3D>& points, const lv_math::types::Pose3D& pose) override;

private:
    void ResetResult();

    Eigen::MatrixXd MakeDescriptor(const std::vector<lv_math::types::Point3D>& points);

    void AddLoopFrame(const lv_math::types::Pose3D &pose, const Eigen::MatrixXd &curr_descriptor);

    bool FindNearestNeighbor(const Eigen::MatrixXd& descriptor, ScanContextFrame& loop_frame);

    bool CheckContactRatio(const Eigen::MatrixXd &loop_descriptor, const Eigen::MatrixXd &curr_descriptor);

    Eigen::MatrixXd AlignasDescriptor(const Eigen::MatrixXd &loop_descriptor, const Eigen::MatrixXd &curr_descriptor);
private:
    Param m_param;
    std::unique_ptr<ScanContextKdtree> m_frame_kd_tree;
};





} // namespace loop_closure
} // namespace front_end

#endif // __SCAN_CONTEXT_H__