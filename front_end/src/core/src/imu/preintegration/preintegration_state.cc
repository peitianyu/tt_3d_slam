#include "preintegration_state.h"

namespace front_end{
namespace imu{

PreintegrationState::PreintegrationState(lv_math::types::Time dt, Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d r)
    : sum_dt(dt), pos(p), vel(v), rot(Eigen::AngleAxisd(r.norm(), r.normalized())) {}

PreintegrationState::PreintegrationState(lv_math::types::Time dt, Eigen::Matrix<double, 9, 1> state)
{
    sum_dt = dt;
    pos = state.head<3>(0);
    vel = state.segment<3>(3);
    rot = Eigen::AngleAxisd(state.segment<3>(6).norm(), state.segment<3>(6).normalized());
}

PreintegrationState::PreintegrationState(const PreintegrationState& other)
{
    sum_dt = other.sum_dt;
    pos = other.pos;
    vel = other.vel;
    rot = other.rot;
}

Eigen::Matrix<double, 9, 1> PreintegrationState::GetState() const
{
    Eigen::Matrix<double, 9, 1> state;
    state.head<3>(0) = pos;
    state.segment<3>(3) = vel;
    Eigen::AngleAxisd r = Eigen::AngleAxisd(rot);
    state.segment<3>(6) = r * r.axis();
    return state;
}


} // namespace imu
} // namespace front_end

