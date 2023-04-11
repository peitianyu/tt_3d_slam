#ifndef __LOOP_CLOSURE_BASE_H__
#define __LOOP_CLOSURE_BASE_H__

#include<vector>
#include"types/pose3d.h"

namespace front_end{
namespace loop_closure{

struct LoopClosureResult
{
    bool found;
    lv_math::types::Pose3D loop_pose;

    LoopClosureResult(bool f = false, lv_math::types::Pose3D d_p = lv_math::types::Pose3D())
        : found(f), loop_pose(d_p) {}
};

class LoopClosureBase
{
public:
    LoopClosureBase() : m_result(LoopClosureResult()) {}

    virtual bool Match(const std::vector<lv_math::types::Point3D>& points, const lv_math::types::Pose3D& pose) = 0;

    const LoopClosureResult& GetResult() const {return m_result;}
protected:
    LoopClosureResult m_result;
};


} // loop_closure
} // front_end







#endif // __LOOP_CLOSURE_BASE_H__