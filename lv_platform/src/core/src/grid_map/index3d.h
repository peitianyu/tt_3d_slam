#ifndef __TYPES_MAP_INDEX_H__
#define __TYPES_MAP_INDEX_H__

#include "types/point3d.h"


namespace grid_map{

struct Index3D
{
    double resolution;
	Eigen::Vector3i index;

    Index3D(const Eigen::Vector3i& index = Eigen::Vector3i::Zero(), const double& res = 0.05);

    Index3D(const types::Point3D& point, const double& res = 0.05);

	types::Point3D Index2Point() const;

    const int& x() const;
    const int& y() const;
    const int& z() const;

    bool operator==(const Index3D &other) const;

    void operator+=(const Index3D &other);

    Index3D operator+(const Index3D &other) const;
};

} // namespace grid_map


#endif // __TYPES_MAP_INDEX_H__