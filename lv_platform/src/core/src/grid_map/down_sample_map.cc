#include "down_sample_map.h"


namespace grid_map{


DownSampleMap::DownSampleMap(const GridMapBase &grid_map)
{
    m_data = grid_map.GetData();
    m_map_limit = grid_map.GetMapLimit();
    m_resolution = grid_map.GetResolution();
}

DownSampleMap::DownSampleMap(const DownSampleMap &down_map)
{
    m_resolution = down_map.GetResolution() * 2.0;
    m_data.clear();
    ResetMapLimit();

    for(const auto& iter: down_map.GetData()){
        Index3D cell_index(iter.first.Index2Point(), iter.first.resolution * 2.0);

        if (m_data.find(cell_index) == m_data.end()){
            UpdateMapLimit(cell_index);
            m_data[cell_index] = iter.second;
        }else{
            m_data[cell_index] = std::max(m_data[cell_index], iter.second);
        }
    }
}



} // namespace grid_map


