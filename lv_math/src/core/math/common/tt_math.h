#ifndef __COMMON_MATH_H__
#define __COMMON_MATH_H__

#include <sys/time.h>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace common{

#define DEGREE_TO_RADIAN (0.01745329251994329576923690768489)
#define RADIAN_TO_DEGREE (57.295779513082320876798154814105)

template<typename T>
bool equal(const T& a, const T& b, const T& eps = 1e-6) { return std::abs(a - b) < eps;}

Eigen::Matrix3d Hat(const Eigen::Vector3d& v);

Eigen::Matrix3d SO3Exp(const Eigen::Vector3d& w);

Eigen::Matrix4d DeltaX2T(const Eigen::Matrix<double, 6, 1>& delta_x);

Eigen::Vector3d NormalizeEuler(const Eigen::Vector3d &v);

Eigen::Matrix3d EulerAnglesToMatrix(const Eigen::Vector3d &angles);

} // namespace common

#endif // __COMMON_MATH_H__