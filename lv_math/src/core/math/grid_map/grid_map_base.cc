#include"grid_map_base.h"

namespace grid_map{


GridMapBase::GridMapBase(double resolution) : m_resolution(resolution)
{
    m_data.clear();
    ResetMapLimit();
}

const std::unordered_map<Index3D, double, index_hash_fn> &GridMapBase::GetData() const
{
    return m_data;
}

bool GridMapBase::IsValid(const Index3D& cell_index) const
{
    if(m_map_limit[0].x() > cell_index.x() || m_map_limit[1].x() < cell_index.x())
        return false;
    
    if(m_map_limit[0].y() > cell_index.y() || m_map_limit[1].y() < cell_index.y())
        return false;

    if(m_map_limit[0].z() > cell_index.z() || m_map_limit[1].z() < cell_index.z())
        return false;

    return true;
}

const std::array<Index3D, 2>& GridMapBase::GetMapLimit() const
{
    return m_map_limit;
}

double GridMapBase::GetCellProb(const Index3D& cell_index) const
{
    if(!IsValid(cell_index)) 
        return 0.5;
    
    double odds = std::exp(GetCellLogOdds(cell_index));
    double prob = (odds / (odds + 1));
    return std::isnan(prob) ? 0.5 : prob;
}

void GridMapBase::UpdateByScan(types::Pose3D key_pose, const std::vector<types::Point3D>& point_cloud)
{
    Index3D begin_point(key_pose.Point(), m_resolution);

    for (const types::Point3D& point : point_cloud)
    {
        Index3D end_point(key_pose.TransformAdd(point), m_resolution);

        UpdateMapLimit(end_point);
        
        if(!(begin_point == end_point))
            InverseModel(begin_point, end_point);
    }
}

void GridMapBase::SetMapLimit(const std::array<Index3D, 2>& map_limit)
{
    m_map_limit = map_limit;
}

void GridMapBase::AddMapData(const Index3D& index, const double& log_odds)
{
    m_data[index] = log_odds;
}

void GridMapBase::Clear()
{
    m_data.clear();
}

double GridMapBase::GetResolution() const 
{
    return m_resolution;
}

void GridMapBase::InverseModel(const Index3D &begin_point, const Index3D &end_point)
{
    BresenhamCellOccupied(end_point);

    BresenhamCellFree(begin_point, end_point);
}

void GridMapBase::BresenhamCellOccupied(const Index3D &end_point)
{
    SetCellOccupied(end_point);
}

void GridMapBase::BresenhamCellFree(const Index3D &begin_point, const Index3D &end_point)
{
    BrasenHam(begin_point, end_point);
}

void GridMapBase::BrasenHam(const Index3D &begin_point, const Index3D &end_point)
{
    int dx = end_point.index(0) - begin_point.index(0);
    int dy = end_point.index(1) - begin_point.index(1);
    int dz = end_point.index(2) - begin_point.index(2);
    int x = begin_point.index(0);
    int y = begin_point.index(1);
    int z = begin_point.index(2);
    int x_inc = (dx < 0) ? -1 : 1;
    int y_inc = (dy < 0) ? -1 : 1;
    int z_inc = (dz < 0) ? -1 : 1;
    int longest = abs(dx);
    int shortest = abs(dy);
    if (longest < abs(dy))
    {
        longest = abs(dy);
        shortest = abs(dx);
    }
    if (longest < abs(dz))
    {
        longest = abs(dz);
        shortest = abs(dy);
    }
    int numerator = longest >> 1;
    for (int i = 0; i <= longest; i++)
    {
        Index3D cell_index(Eigen::Vector3i(x, y, z));
        if (m_data.find(cell_index) != m_data.end()){
            SetCellFree(cell_index);

            constexpr double free_thr = 0.4;
            if (GetCellProb(cell_index) < free_thr)
                m_data.erase(cell_index);
        }
            
        numerator += shortest;
        if (!(numerator < longest))
        {
            numerator -= longest;
            x += x_inc;
            y += y_inc;
            z += z_inc;
        }
        else
        {
            x += x_inc;
            y += y_inc;
            z += z_inc;
        }
    }
}

void GridMapBase::UpdateMapLimit(const Index3D &index)
{
    m_map_limit[0].index(0) = std::min(m_map_limit[0].index(0), index.index(0));
    m_map_limit[0].index(1) = std::min(m_map_limit[0].index(1), index.index(1));
    m_map_limit[0].index(2) = std::min(m_map_limit[0].index(2), index.index(2));
    m_map_limit[1].index(0) = std::max(m_map_limit[1].index(0), index.index(0));
    m_map_limit[1].index(1) = std::max(m_map_limit[1].index(1), index.index(1));
    m_map_limit[1].index(2) = std::max(m_map_limit[1].index(2), index.index(2));
}

void GridMapBase::SetCellOccupied(const Index3D& cell_index)
{
    constexpr double log_odds_p_occ = 0.6;
    
    if (m_data.find(cell_index) != m_data.end())
        m_data[cell_index] += log_odds_p_occ;
    else{
        m_data[cell_index] = log_odds_p_occ;
    } 
}

void GridMapBase::SetCellFree(const Index3D& cell_index)
{
    constexpr double log_odds_p_free = 0.4;
    
    if (m_data.find(cell_index) != m_data.end())
        m_data[cell_index] -= log_odds_p_free;
    else
        m_data[cell_index] = -log_odds_p_free;
}

double GridMapBase::GetCellLogOdds(const Index3D& cell_index) const
{
    
    if (m_data.find(cell_index) == m_data.end())
        return 0.0;

    return m_data.find(cell_index)->second;
}

void GridMapBase::ResetMapLimit()
{
    constexpr int int_max = std::numeric_limits<int>::max();
    constexpr int int_min = std::numeric_limits<int>::min();
    m_map_limit[0] = Index3D(Eigen::Vector3i(int_max, int_max, int_max), m_resolution);
    m_map_limit[1] = Index3D(Eigen::Vector3i(int_min, int_min, int_min), m_resolution);
}

} // namespace grid_map

