#ifndef __GPS_CONVERTOR_H__
#define __GPS_CONVERTOR_H__

#include <Eigen/Dense>
#include "GeographicLib/LocalCartesian.hpp"

namespace front_end{

class GpsConverter
{
public:
    GpsConverter(const Eigen::Vector3d& origin_lla) : origin_lla_(origin_lla) {}

    void SetOrigin(const Eigen::Vector3d& origin_lla) { origin_lla_ = origin_lla; }

    void LLAToENU(const Eigen::Vector3d& lla, Eigen::Vector3d& enu)
    {
        static GeographicLib::LocalCartesian local_cartesian; 
        local_cartesian.Reset(origin_lla_(0), origin_lla_(1), origin_lla_(2));  
        local_cartesian.Forward(lla(0), lla(1), lla(2), enu(0), enu(1), enu(2));
    }

    Eigen::Vector3d LLAToENU(const Eigen::Vector3d& lla)
    {
        Eigen::Vector3d enu;
        LLAToENU(lla, enu);
        return enu;
    }

    void ENUToLLA(const Eigen::Vector3d& enu, Eigen::Vector3d& lla)
    {
        static GeographicLib::LocalCartesian local_cartesian;
        local_cartesian.Reset(origin_lla_(0), origin_lla_(1), origin_lla_(2));
        local_cartesian.Reverse(enu(0), enu(1), enu(2), lla(0), lla(1), lla(2));
    }

    Eigen::Vector3d ENUToLLA(const Eigen::Vector3d& enu)
    {
        Eigen::Vector3d lla;
        ENUToLLA(enu, lla);
        return lla;
    }
private:
    Eigen::Vector3d origin_lla_;
};

} // namespace front_end

#endif // __GPS_CONVERTOR_H__