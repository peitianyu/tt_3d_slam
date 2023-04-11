#ifndef __SCAN_CONTEXT_KDTREE_H__
#define __SCAN_CONTEXT_KDTREE_H__

#include "scan_context_type.h"
#include <Eigen/Dense>

namespace front_end {
namespace loop_closure {

struct ScanContextKdtree
{
    ScanContextKdtree() {}

    ScanContextKdtree(const std::vector<ScanContextFrame> &frames)
    {
        for(const ScanContextFrame& frame: frames)
            AddScanContextFrame(frame);
    }

    void AddScanContextFrame(const ScanContextFrame& frame) { 
        frames.push_back(frame);
        pts.push_back(frame.descriptor.rowwise().sum());
    }
    
    std::vector<Eigen::VectorXd> pts;
    std::vector<ScanContextFrame> frames;

    inline size_t kdtree_get_point_count() const { return pts.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const { return pts[idx](dim); }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false;}
};

} // namespace loop_closure
} // namespace front_end

#endif // __SCAN_CONTEXT_KDTREE_H__