#include"index3d.h"


namespace grid_map{

Index3D::Index3D(const Eigen::Vector3i& index, const double& res)
    : resolution(res), index(index)
{}

Index3D::Index3D(const types::Point3D& point, const double& res)
    : resolution(res)
{
    index(0) = std::round(point.x() / resolution);
    index(1) = std::round(point.y() / resolution);
    index(2) = std::round(point.z() / resolution);
}

types::Point3D Index3D::Index2Point() const { return index.cast<double>() * resolution;}

const int& Index3D::x() const {return index(0);}
const int& Index3D::y() const {return index(1);}
const int& Index3D::z() const {return index(2);}

bool Index3D::operator==(const Index3D &other) const
{
    if(index(0) != other.index(0))
        return false;

    if(index(1) != other.index(1))
        return false;
        
    return index(2) == other.index(2);
}

void Index3D::operator+=(const Index3D &other)
{
    assert(fabs(resolution - other.resolution) < 1e-6 && "Resolution is not equal!" );
    index += other.index;
}

Index3D Index3D::operator+(const Index3D &other) const
{
    assert(fabs(resolution - other.resolution) < 1e-6 && "Resolution is not equal!" );
    Eigen::Vector3i new_index = index + other.index;
    return Index3D(new_index, other.resolution);
}


} // namespace grid_map

