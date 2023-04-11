#ifndef __GRID_MAP_DOWN_SAMPLE_H__
#define __GRID_MAP_DOWN_SAMPLE_H__

#include <Eigen/Core>
#include "grid_map_base.h"

namespace grid_map{

class DownSampleMap: public GridMapBase
{
public:
    DownSampleMap() = default;

    DownSampleMap(const GridMapBase &grid_map);

    DownSampleMap(const DownSampleMap &down_map);
};

} // namespace grid_map


#endif // __GRID_MAP_DOWN_SAMPLE_H__
