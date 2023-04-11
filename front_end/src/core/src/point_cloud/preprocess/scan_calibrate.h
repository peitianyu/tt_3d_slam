#ifndef __FRONT_END_SCAN_CALIBRATE_H__
#define __FRONT_END_SCAN_CALIBRATE_H__

#include <vector>

#include "../types/point_cloud.h"
#include "imu/types/imu.h" 
#include "types/rot3d.h"
#include "common/log.h"


namespace front_end{
namespace point_cloud{

class ScanCalibrate
{
public:
    ScanCalibrate() = default;

    bool Calibrate(const front_end::point_cloud::TimedPointCloudPCD& timed_point_cloud);

    void UpdateImu(const front_end::imu::Imu& imu);

    const front_end::point_cloud::TimedPointCloudPCD& GetCorrectedPointCloud();
private:
    bool CheckTimestamp(const front_end::point_cloud::TimedPointCloudPCD& timed_point_cloud);

    void EraseInvalidImu(lv_math::types::Time point_cloud_start_time);

    void IntegrateImu(lv_math::types::Time point_cloud_end_time);

    void Correct(const front_end::point_cloud::TimedPointCloudPCD& timed_point_cloud);

    lv_math::types::Rot3D GetInterpolatedRot(lv_math::types::Time time_stamp);
private:
    std::vector<front_end::imu::Imu> m_imu_que;
    std::vector<std::pair<lv_math::types::Time, Eigen::Vector3d>> m_calibrate_rot;
    front_end::point_cloud::TimedPointCloudPCD m_corrected_point_cloud;
};

} // namespace point_cloud
} // namespace front_end

#endif // __FRONT_END_SCAN_CALIBRATE_H__