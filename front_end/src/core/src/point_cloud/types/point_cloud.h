#ifndef __POINT_CLOUD_H__
#define __POINT_CLOUD_H__

#include <iostream>
#include <Eigen/Core>
#include <map>
#include <vector>
#include"point_cloud_type.h"
#include"types/time.h"

namespace front_end{
namespace point_cloud{

template<class PointType>
class PointCloud
{
public:
    PointCloud() = default;

    explicit PointCloud(const std::map<uint, std::vector<PointType>> &data) : m_point_cloud(data){}

    void AddPoint(const uint& line_num, const PointType& point) { m_point_cloud[line_num].push_back(point);} // 编号

    void AddPoints(const uint& line_num, const std::vector<PointType>& ps)
       { m_point_cloud[line_num].insert(m_point_cloud[line_num].end(), ps.begin(), ps.end());}

    const std::map<uint, std::vector<PointType>>& GetPointCloud() const { return m_point_cloud;}

    const std::vector<PointType>& GetPoints(const uint& line_num) const { return m_point_cloud.at(line_num);}

    std::vector<PointType> GetPoints() const
    {
        std::vector<PointType> points;
        for(const auto& p : m_point_cloud)
            points.insert(points.end(), p.second.begin(), p.second.end());
        return points;
    }

    void Clear() { m_point_cloud.clear(); }
private:
    std::map<uint, std::vector<PointType>> m_point_cloud;
};

using PointCloudPCD = PointCloud<CloudPointType>;

struct TimedPointCloudPCD
{
    lv_math::types::Time time_stamp;
    lv_math::types::Time scan_time;
    PointCloudPCD point_cloud;

    TimedPointCloudPCD(const lv_math::types::Time& t = lv_math::types::Time(), const lv_math::types::Time& s = lv_math::types::Time(), const PointCloudPCD& p = PointCloudPCD())
        : time_stamp(t), scan_time(s), point_cloud(p) {}
};

       
} // namespace point_cloud
} // namespace front_end

#endif // __POINT_CLOUD_H__