#ifndef __TYPES_ROT3D_H__
#define __TYPES_ROT3D_H__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "time.h"


namespace types {

class Rot3D
{
public:
    explicit Rot3D(const Eigen::Quaterniond &q = Eigen::Quaterniond::Identity()) : m_q(q.normalized()) {}
    
    explicit Rot3D(const Eigen::Matrix3d &r_m) : m_q(r_m) { m_q.normalize(); }

    Rot3D(double x, double y, double z) : m_q(  Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX())
                                                * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY())
                                                * Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ())) { m_q.normalize(); }

    Eigen::Vector3d ToEuler() const {return m_q.toRotationMatrix().eulerAngles(0, 1, 2);}

    Eigen::Quaterniond ToQuaternion() const { return m_q; }

    Eigen::Matrix3d ToMatrix() const { return m_q.toRotationMatrix(); }
private:
    Eigen::Quaterniond m_q;
};

struct TimedRot3D
{
    Time time_stamp;
    Rot3D rot;

    TimedRot3D(const Time& time = 0.0, const Rot3D& rot = Rot3D())
        : time_stamp(time), rot(rot)
    {}
};

} // namespace types

#endif // __TYPES_ROT3D__