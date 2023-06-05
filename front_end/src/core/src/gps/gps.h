#ifndef __GPS_H__
#define __GPS_H__

#include <Eigen/Core>
#include <memory>
#include "types/time.h"

namespace front_end{

struct Gps
{
    types::Time time_stamp;

    Eigen::Vector3d lla; 
    Eigen::Matrix3d cov;
};

} // namespace front_end

#endif // __GPS_H__