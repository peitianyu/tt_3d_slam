#ifndef __SCAN_MATCH_BASE_H__
#define __SCAN_MATCH_BASE_H__

#include <iostream>
#include <vector>
#include "types/pose3d.h"

using namespace lv_math::types;

namespace front_end{
namespace point_cloud{

struct ScanMatchResult
{
    bool is_converged;
    double score;
    double error;
    Pose3D robot_pose;

    ScanMatchResult(bool is_s = false, double s = 0.0f, double e = 1e6f, Pose3D p = Pose3D())
        : is_converged(is_s), score(s), error(e), robot_pose(p) {}
    
    friend std::ostream& operator<<(std::ostream& os, const ScanMatchResult& result)
    {
        os << result.is_converged << " " << result.score << " " << result.error << " " << result.robot_pose;
        return os;
    }
};

class ScanMatchBase
{
public:
    ScanMatchBase() : m_result(ScanMatchResult()) {}

    const ScanMatchResult& GetResult() const { return m_result; }
protected:
    void ResetResult(const Pose3D& prior_pose)
    {
        m_result.robot_pose = prior_pose;
        m_result.is_converged = false;
        m_result.error = 1e6f;
        m_result.score = 0.0f;
    }

    std::vector<Point3D> TransformPointCloud(const std::vector<Point3D>& ps, const Pose3D& pose)
    {
        std::vector<Point3D> transformed_ps;
        for(const auto& p : ps)
            transformed_ps.push_back(pose .TransformAdd(p));
        return transformed_ps;
    }
protected:
    ScanMatchResult m_result;
};

} // namespace point_cloud
} // namespace front_end

#endif // __SCAN_MATCH_BASE_H__