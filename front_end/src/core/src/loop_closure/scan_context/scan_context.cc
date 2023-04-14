#include"scan_context.h"
#include "common/tt_log.h"
#include "common/tt_assert.h"

namespace front_end {
namespace loop_closure {

#define RING_NUM    20
#define SECTOR_NUM  60


ScanContext::ScanContext(const Param& param) : LoopClosureBase(), m_param(param)
{
    m_frame_kd_tree = std::unique_ptr<ScanContextKdtree>(new ScanContextKdtree());
}

bool ScanContext::Match(const std::vector<types::Point3D>& points, const types::Pose3D& pose)
{
    Eigen::MatrixXd curr_descriptor = MakeDescriptor(points);

    AddLoopFrame(pose, curr_descriptor);

    if(m_frame_kd_tree->kdtree_get_point_count() < m_param.match_thr)
        return false;

    ResetResult();
    ScanContextFrame loop_frame;
    if(!FindNearestNeighbor(curr_descriptor, loop_frame))
        return false;
    
    if(!CheckContactRatio(loop_frame.descriptor, curr_descriptor))
        return false;

    m_result.found = true;
    m_result.loop_pose = loop_frame.pose;

    return true;
}

void ScanContext::ResetResult()
{
    m_result = LoopClosureResult();
}

Eigen::MatrixXd ScanContext::MakeDescriptor(const std::vector<types::Point3D>& points)
{
    Eigen::MatrixXd descriptor = Eigen::MatrixXd::Zero(RING_NUM, SECTOR_NUM);

    for(const auto& point : points)
    {
        double dist = std::sqrt(point(0) * point(0) + point(1) * point(1));
        double angle = std::atan2(point(1), point(0)) * RADIAN_TO_DEGREE + 180;

        if(dist < m_param.min_range || dist > m_param.max_range)
            continue;
        
        int ring = std::max(std::min(RING_NUM - 1, static_cast<int>(round((dist / m_param.max_range) * RING_NUM))), 0);
        int sector = std::max(std::min(SECTOR_NUM - 1, static_cast<int>(round((angle / 360.0f) * SECTOR_NUM))), 0);

        descriptor(ring, sector) += point(2);
    }

    return descriptor;
}

void ScanContext::AddLoopFrame(const types::Pose3D &pose, const Eigen::MatrixXd &curr_descriptor)
{
    m_frame_kd_tree->AddScanContextFrame(ScanContextFrame(m_frame_kd_tree->kdtree_get_point_count(), pose, curr_descriptor));
}

bool ScanContext::FindNearestNeighbor(const Eigen::MatrixXd& descriptor, ScanContextFrame& loop_frame)
{
    using kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, ScanContextKdtree>, ScanContextKdtree, RING_NUM>;
    
    ScanContextKdtree frame_kd_tree = *m_frame_kd_tree;
    frame_kd_tree.pts.assign(frame_kd_tree.pts.begin(), frame_kd_tree.pts.end() - m_param.num_exclude_recent);
    kd_tree_t scan_context_kd_tree(RING_NUM, frame_kd_tree, {10});

    const size_t num_results = 1;
    size_t ret_index;
    double out_dist_sqr;
    nanoflann::KNNResultSet<double> result_set(num_results);
    result_set.init(&ret_index, &out_dist_sqr);

    Eigen::VectorXd ref_ring = descriptor.rowwise().sum();
    scan_context_kd_tree.findNeighbors(result_set, &ref_ring(0));

    std::cout << "out_dist_sqr: " << out_dist_sqr << std::endl;
    if (out_dist_sqr > m_param.search_radius)
        return false;

    loop_frame = scan_context_kd_tree.dataset_.frames[ret_index];
    
    return true;
}

bool ScanContext::CheckContactRatio(const Eigen::MatrixXd &loop_descriptor, const Eigen::MatrixXd &curr_descriptor)
{
    tt_assert(loop_descriptor.rows() == RING_NUM && loop_descriptor.cols() == SECTOR_NUM);
    Eigen::MatrixXd aligned_descriptor = AlignasDescriptor(loop_descriptor, curr_descriptor);

    // std::cout<< loop_descriptor <<std::endl;
    // std::cout<<"-----------------------------------------------------"<<std::endl;
    // std::cout << aligned_descriptor << std::endl;

    std::cout << "contact_ratio: " << (aligned_descriptor - loop_descriptor).norm() << std::endl;

    return (aligned_descriptor - loop_descriptor).norm() < m_param.contact_ratio_thr;
}

Eigen::MatrixXd ScanContext::AlignasDescriptor(const Eigen::MatrixXd &loop_descriptor, const Eigen::MatrixXd &curr_descriptor)
{
    Eigen::VectorXd loop_sector_vector = loop_descriptor.colwise().sum().transpose();
    Eigen::VectorXd curr_sector_vector = curr_descriptor.colwise().sum().transpose();

    Eigen::VectorXd tmp_vector = curr_sector_vector;

    double min_diff = std::numeric_limits<double>::max();
    int min_idx = 0;
    for(uint i = 0; i < SECTOR_NUM; i++)
    {
        tmp_vector.setZero();
        tmp_vector.segment(0, SECTOR_NUM - i) = curr_sector_vector.segment(i, SECTOR_NUM - i);
        tmp_vector.segment(SECTOR_NUM - i, i) = curr_sector_vector.segment(0, i);

        double diff = (tmp_vector - loop_sector_vector).norm();
        if(diff < min_diff){
            min_diff = diff;
            min_idx = i;
        }
    }

    tmp_vector.setZero();
    tmp_vector.segment(0, SECTOR_NUM - min_idx) = curr_sector_vector.segment(min_idx, SECTOR_NUM - min_idx);
    tmp_vector.segment(SECTOR_NUM - min_idx, min_idx) = curr_sector_vector.segment(0, min_idx);

    Eigen::MatrixXd aligned_descriptor = Eigen::MatrixXd::Zero(RING_NUM, SECTOR_NUM);    
    // 根据最小差值的索引，将当前帧的描述子对齐到回环帧的描述子
    aligned_descriptor.block(0, 0, RING_NUM, SECTOR_NUM - min_idx) = curr_descriptor.block(0, min_idx, RING_NUM, SECTOR_NUM - min_idx);
    aligned_descriptor.block(0, SECTOR_NUM - min_idx, RING_NUM, min_idx) = curr_descriptor.block(0, 0, RING_NUM, min_idx);

    return aligned_descriptor;
}






} // namespace loop_closure
} // namespace front_end

