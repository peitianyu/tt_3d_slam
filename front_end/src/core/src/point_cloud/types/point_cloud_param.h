#ifndef __POINT_CLOUD_PARAM_H__
#define __POINT_CLOUD_PARAM_H__

#include<iostream>


namespace front_end{
namespace point_cloud{

struct PointCloudParam
{
    bool use;
    uint num;
    std::string name;
    uint line_num;
    uint scan_size;
    double min_range;
    double max_range;
    double downsample_rate;

    PointCloudParam(const bool& u = false, const uint& _num = 0, const std::string& n = "LASER"
    , const uint& l_n = 16, const uint& s_s = 1800, const double& min = 1.0, const double& max = 100.0, const double& d_s = 1.0)
        : use(u), num(_num), name(n), line_num(l_n), scan_size(s_s), min_range(min), max_range(max), downsample_rate(d_s){}

};


} // namespace point_cloud
} // namespace front_end



#endif // __POINT_CLOUD_PARAM_H__