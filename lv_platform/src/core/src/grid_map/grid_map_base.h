#ifndef __TYPES_GRID_MAP_BASE_H__
#define __TYPES_GRID_MAP_BASE_H__


#include "index3d.h"
#include "types/pose3d.h"
#include <unordered_map>
#include <vector>
#include <mutex>


namespace grid_map{

struct index_hash_fn
{
    std::size_t operator() (const Index3D& index) const 
    {
        return std::hash<int>()(index.x()) ^ std::hash<int>()(index.y()) ^ std::hash<int>()(index.z());
    }
};


class GridMapBase
{
public:
    GridMapBase(double resolution = 0.05);

    GridMapBase& operator=(const GridMapBase&) = default;

    const std::unordered_map<Index3D, double, index_hash_fn> &GetData() const;

    bool IsValid(const Index3D& cell_index) const;

    const std::array<Index3D, 2>& GetMapLimit() const;

    double GetCellProb(const Index3D& cell_index) const;

    void UpdateByScan(types::Pose3D key_pose, const std::vector<types::Point3D>& point_cloud);

    void SetMapLimit(const std::array<Index3D, 2>& map_limit);

    void AddMapData(const Index3D& index, const double& log_odds);

    void Clear();

    double GetResolution() const;

protected:
    void InverseModel(const Index3D &begin_point, const Index3D &end_point);

    void BresenhamCellOccupied(const Index3D &end_point);

    void BresenhamCellFree(const Index3D &begin_point, const Index3D &end_point);

    void BrasenHam(const Index3D &begin_point, const Index3D &end_point);

    void UpdateMapLimit(const Index3D &index);

    void SetCellOccupied(const Index3D& cell_index);

    void SetCellFree(const Index3D& cell_index);

    double GetCellLogOdds(const Index3D& cell_index) const;

    void ResetMapLimit();
protected:
    double m_resolution;
    std::unordered_map<Index3D, double, index_hash_fn> m_data;
    std::array<Index3D, 2> m_map_limit;
};


} // namespace grid_map


#endif // __TYPES_GRID_MAP_BASE_H__