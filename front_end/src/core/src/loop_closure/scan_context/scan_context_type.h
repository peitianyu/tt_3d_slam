#ifndef __SCAN_CONTEXT_TYPE_H__
#define __SCAN_CONTEXT_TYPE_H__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include "types/pose3d.h"


namespace front_end {
namespace loop_closure {

struct ScanContextFrame
{
    int id;
    types::Pose3D pose;
    Eigen::MatrixXd descriptor;

    ScanContextFrame(int i, const types::Pose3D& p, const Eigen::MatrixXd& d)
        : id(i), pose(p), descriptor(d) {}
    
    ScanContextFrame() {}
};

} // namespace loop_closure
} // namespace front_end

#endif // __SCAN_CONTEXT_TYPE_H__