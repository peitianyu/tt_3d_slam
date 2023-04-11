#include"math.h"
#include<Eigen/Geometry>

namespace common{

Eigen::Matrix3d Hat(const Eigen::Vector3d& v)
   { return (Eigen::Matrix3d() << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0).finished();}

Eigen::Matrix3d SO3Exp(const Eigen::Vector3d& w)
{
   Eigen::Matrix3d R;
   double theta = w.norm();
   Eigen::Vector3d w_normalized = w.normalized();
   R = std::cos(theta) * Eigen::Matrix3d::Identity()
      + (1.0 - std::cos(theta)) * w_normalized * w_normalized.transpose()
      + std::sin(theta) * Hat(w_normalized);
   return R;
}

Eigen::Vector3d NormalizeEuler(const Eigen::Vector3d &v)
{
   Eigen::Vector3d v_norm = v;
   for(int i = 0; i < 3; i++){
      while(v_norm(i) > M_PI){ v_norm(i) -= 2 * M_PI;}
      while(v_norm(i) < -M_PI){ v_norm(i) += 2 * M_PI;}
   }
   return v_norm;
}

Eigen::Matrix3d EulerAnglesToMatrix(const Eigen::Vector3d &angles)
{
   return Eigen::Matrix3d(Eigen::AngleAxisd(angles[2], Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(angles[1], Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(angles[0], Eigen::Vector3d::UnitX()));
}

} // namespace common

