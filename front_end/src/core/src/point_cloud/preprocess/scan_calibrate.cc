#include"scan_calibrate.h"

namespace front_end{
namespace point_cloud{

bool ScanCalibrate::Calibrate(const front_end::point_cloud::TimedPointCloudPCD& timed_point_cloud)
{
    if (!CheckTimestamp(timed_point_cloud))
        return false;

    EraseInvalidImu(timed_point_cloud.time_stamp);

    IntegrateImu(timed_point_cloud.time_stamp + timed_point_cloud.scan_time);

    Correct(timed_point_cloud);
    return true;
}

void ScanCalibrate::UpdateImu(const front_end::imu::Imu& imu)
{
    m_imu_que.push_back(imu);
}

const front_end::point_cloud::TimedPointCloudPCD& ScanCalibrate::GetCorrectedPointCloud()
{
    return m_corrected_point_cloud;
}

bool ScanCalibrate::CheckTimestamp(const front_end::point_cloud::TimedPointCloudPCD& timed_point_cloud)
{
    if (m_imu_que.empty())
        return false;
    
    if (timed_point_cloud.time_stamp < m_imu_que.front().time_stamp)
        return false;

    if ((timed_point_cloud.time_stamp + timed_point_cloud.scan_time) > m_imu_que.back().time_stamp)
        return false;

    return true;
}

void ScanCalibrate::EraseInvalidImu(lv_math::types::Time point_cloud_start_time)
{
    while (m_imu_que.front().time_stamp < point_cloud_start_time)
        m_imu_que.erase(m_imu_que.begin());
}

void ScanCalibrate::IntegrateImu(lv_math::types::Time point_cloud_end_time)
{
    m_calibrate_rot.clear();
    m_calibrate_rot.push_back(std::make_pair(m_imu_que.front().time_stamp, Eigen::Vector3d::Zero()));

    for(uint i = 1; m_imu_que[i-1].time_stamp < point_cloud_end_time; i++)
    {
        lv_math::types::Time d_t = m_imu_que[i].time_stamp - m_imu_que[i - 1].time_stamp;
        Eigen::Vector3d d_rot = m_imu_que[i - 1].gyr * d_t;
        m_calibrate_rot.push_back(std::make_pair(m_imu_que[i].time_stamp, m_calibrate_rot.back().second + d_rot));
    }
}

void ScanCalibrate::Correct(const front_end::point_cloud::TimedPointCloudPCD& timed_point_cloud)
{
    m_corrected_point_cloud = front_end::point_cloud::TimedPointCloudPCD();
    m_corrected_point_cloud.time_stamp = timed_point_cloud.time_stamp;
    m_corrected_point_cloud.scan_time = timed_point_cloud.scan_time;

    for (auto& iter : timed_point_cloud.point_cloud.GetPointCloud())
    {
        uint line_num = iter.first;
        for(const auto& p: iter.second)
        {
            lv_math::types::Rot3D rot = GetInterpolatedRot(p.timed_point.time_stamp);
            lv_math::types::Point3D point = rot.ToQuaternion().conjugate() * p.timed_point.point;

            CloudPointType cloud_point_type(p.index, lv_math::types::TimedPoint3D(p.timed_point.time_stamp, point));
            m_corrected_point_cloud.point_cloud.AddPoint(line_num, cloud_point_type);
        }
    }
}

lv_math::types::Rot3D ScanCalibrate::GetInterpolatedRot(lv_math::types::Time time_stamp)
{
    for(uint i = 1; i < m_calibrate_rot.size(); i++)
    {
        if (time_stamp < m_calibrate_rot[i].first)
        {
            Eigen::Vector3d front_rot = m_calibrate_rot[i - 1].second;
            Eigen::Vector3d back_rot = m_calibrate_rot[i].second;

            double front_ratio = (m_calibrate_rot[i].first - time_stamp) / (m_calibrate_rot[i].first - m_calibrate_rot[i - 1].first);
            double back_ratio = (time_stamp - m_calibrate_rot[i - 1].first) / (m_calibrate_rot[i].first - m_calibrate_rot[i - 1].first);

            Eigen::Vector3d rot = front_rot * front_ratio + back_rot * back_ratio;
            return lv_math::types::Rot3D(rot(0), rot(1), rot(2));
        }
    }
    return lv_math::types::Rot3D();
}

} // namespace point_cloud
} // namespace front_end

