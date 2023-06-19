#ifndef __POINT_CLOUD_H__
#define __POINT_CLOUD_H__

#include <iostream>
#include <Eigen/Core>
#include <map>
#include <vector>

namespace front_end{
namespace point_cloud{

template<class PointType>
class PointCloud
{
public:
    PointCloud() = default;

    explicit PointCloud(const std::vector<PointType> &data) : m_point_cloud(data){}

    void AddPoint(const PointType& point) { m_point_cloud.push_back(point); }

    void AddPoints(const std::vector<PointType>& ps) { m_point_cloud.insert(m_point_cloud.end(), ps.begin(), ps.end());}

    const std::vector<PointType>& GetPointCloud() const { return m_point_cloud;}

    void Clear() { m_point_cloud.clear(); }
private:
    std::vector<PointType> m_point_cloud;
};

struct RgbPoint3D
{
    Eigen::Vector3d pos;
    Eigen::Vector3i rgb;

    RgbPoint3D(Eigen::Vector3d p = Eigen::Vector3d(), Eigen::Vector3i c = Eigen::Vector3i())
        : pos(p), rgb(c) {}
};

using PointCloudRGB = PointCloud<RgbPoint3D>;
using PointCloudXYZ = PointCloud<Eigen::Vector3d>;
       
} // namespace camera
} // namespace front_end

#endif // __POINT_CLOUD_H__