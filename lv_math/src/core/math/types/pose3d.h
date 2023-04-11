#ifndef __TYPES_POSE3D_H__
#define __TYPES_POSE3D_H__

#include<iostream>
#include<fstream>
#include "rot3d.h"
#include "point3d.h"
#include "time.h"
#include "common/tt_math.h"

namespace types {

class Pose3D
{
public:
    Pose3D(const Point3D &point, const Eigen::Matrix3d &r_m)
        : m_point(point), m_rot(r_m)
    {}

    Pose3D(const Point3D &point, const Eigen::Quaterniond &q)
        : m_point(point), m_rot(q)
    {}

    Pose3D(const Point3D &point, const Rot3D &rot)
        : m_point(point), m_rot(rot)
    {}

    Pose3D(const Eigen::Matrix<double, 6, 1> &pose)
        : Pose3D(pose.head(3), common::SO3Exp(pose.tail(3)))
    {}

    Pose3D() : Pose3D(Point3D::Zero(), Rot3D())
    {}

    Point3D Point() const { return m_point; }
    Rot3D Rot() const { return m_rot; }
    size_t Dim() const { return 6; }

    friend std::ostream& operator<<(std::ostream& os, const Pose3D& pose)
    {
        os << pose.m_point.transpose() << " " << pose.m_rot.ToEuler().transpose();
        return os;
    }

    Pose3D TransformAdd(const Pose3D &pose)
    {
        Point3D transition = m_rot.ToMatrix() * pose.Point() + m_point;
        return Pose3D(transition, Rot3D(m_rot.ToQuaternion() * pose.Rot().ToQuaternion()));
    }

    Pose3D TransformFrom(const Pose3D &pose)
    {
        Point3D transition = m_rot.ToMatrix().transpose() * (pose.Point() - m_point);
        return Pose3D(transition, Rot3D(m_rot.ToQuaternion().conjugate() *  pose.Rot().ToQuaternion()));
    }

    Point3D TransformAdd(const Point3D &point) const
    {
        return m_rot.ToMatrix() * point + m_point;
    }

    Point3D TransformFrom(const Point3D &point) const
    {
        return m_rot.ToMatrix().transpose() * (point - m_point);
    }
private:
    Point3D m_point;
    Rot3D m_rot;
};


struct TimedPose3D
{
    Time time_stamp;
    Pose3D pose;

    TimedPose3D(const Time& time = 0.0, const Pose3D& pose = Pose3D())
        : time_stamp(time), pose(pose)
    {}
};


} // namespace types


#endif // __TYPES_POSE3D__