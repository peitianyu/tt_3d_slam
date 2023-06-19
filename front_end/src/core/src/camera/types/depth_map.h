#ifndef __DEPTH_MAP_H__
#define __DEPTH_MAP_H__

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "point_cloud.h"
#include "types/pose3d.h"

namespace front_end{
namespace point_cloud{

class DepthMap
{
public:
    struct Param
    {
        double cx;
        double cy;
        double fx;
        double fy;
        double depth_scale;

        Param(double cx_ = 325.5f, double cy_ = 253.5f, double fx_ = 518.0f, double fy_ = 519.0f, double depth_scale_ = 1000.0f)
            : cx(cx_), cy(cy_), fx(fx_), fy(fy_), depth_scale(depth_scale_) {}
    };

    DepthMap(const Param& param = Param(), const std::string& depth_map_path = "", const std::string& color_img_path = "")
        : m_param(param)
    {
        if(!depth_map_path.empty())
            m_depth_map = cv::imread(depth_map_path, -1);
        if(!color_img_path.empty())
            m_color_img = cv::imread(color_img_path);

        if(m_depth_map.empty() || m_color_img.empty())
        {
            std::cout << "depth map or color image is empty!" << std::endl;
            exit(0);
        }
    }

    DepthMap(const Param& param, const cv::Mat& depth_map, const cv::Mat& color_img)
        : m_param(param), m_depth_map(depth_map), m_color_img(color_img) {}
    
    DepthMap(const Param& param, const PointCloudRGB& pc)
        : m_param(param)
    {
        m_depth_map = cv::Mat::zeros(480, 640, CV_8UC1);
        m_color_img = cv::Mat::zeros(480, 640, CV_8UC3);
        for(auto& p : pc.GetPointCloud())
        {
            Eigen::Vector3d pos = p.pos;
            Eigen::Vector3i rgb = p.rgb;

            int m = pos[1] * m_param.fy / pos[2] + m_param.cy;
            int n = pos[0] * m_param.fx / pos[2] + m_param.cx;
            if(m < 0 || m >= m_depth_map.rows || n < 0 || n >= m_depth_map.cols) continue;
            ushort d = pos[2] * m_param.depth_scale;
            if(m_depth_map.ptr<ushort>(m)[n] == 0 || m_depth_map.ptr<ushort>(m)[n] > d)
            {
                m_depth_map.ptr<ushort>(m)[n] = d;
                m_color_img.ptr<uchar>(m)[n * 3] = rgb[2];
                m_color_img.ptr<uchar>(m)[n * 3 + 1] = rgb[1];
                m_color_img.ptr<uchar>(m)[n * 3 + 2] = rgb[0];
            }
        }
    }

    PointCloudRGB GeneratePointCloudRGB(types::Pose3D pose)
    {
        PointCloudRGB pc;
        for(int m = 0; m < m_depth_map.rows; m++)
        {
            for(int n = 0; n < m_depth_map.cols; n++)
            {
                ushort d = m_depth_map.ptr<ushort>(m)[n];
                if(d == 0) continue; // 为0表示没有测量到
                    
                double x = (n - m_param.cx) / m_param.fx * d / m_param.depth_scale;
                double y = (m - m_param.cy) / m_param.fy * d / m_param.depth_scale;
                double z = d / m_param.depth_scale;

                uint8_t b = m_color_img.ptr<uchar>(m)[n * 3];
                uint8_t g = m_color_img.ptr<uchar>(m)[n * 3 + 1];
                uint8_t r = m_color_img.ptr<uchar>(m)[n * 3 + 2];

                pc.AddPoint(RgbPoint3D(pose.TransformAdd(Eigen::Vector3d(x, y, z)), Eigen::Vector3i(r, g, b)));
            }
        }
        return pc;
    }

    PointCloudXYZ GeneratePointCloudXYZ(types::Pose3D pose)
    {
        PointCloudXYZ pc;
        for(int m = 0; m < m_depth_map.rows; m++)
        {
            for(int n = 0; n < m_depth_map.cols; n++)
            {
                ushort d = m_depth_map.ptr<ushort>(m)[n];
                if(d == 0) continue; // 为0表示没有测量到
                    
                double x = (n - m_param.cx) / m_param.fx * d / m_param.depth_scale;
                double y = (m - m_param.cy) / m_param.fy * d / m_param.depth_scale;
                double z = d / m_param.depth_scale;

                pc.AddPoint(pose.TransformAdd(Eigen::Vector3d(x, y, z)));
            }
        }
        return pc;
    }
private:
    Param m_param;
    cv::Mat m_depth_map;
    cv::Mat m_color_img;
};

} // namespace camera
} // namespace front_end

#endif // __DEPTH_MAP_H__