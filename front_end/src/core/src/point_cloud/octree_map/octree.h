#ifndef __OCTREE_MAP_H__
#define __OCTREE_MAP_H__

#include"types/point3d.h"
#include<vector>

namespace front_end{
namespace point_cloud{

struct OctreeNode
{
    std::vector<types::Point3D> points;
    OctreeNode* children[8];

    OctreeNode()
    {
        for (int i = 0; i < 8; i++)
            children[i] = nullptr;
    }

    ~OctreeNode()
    {
        for (int i = 0; i < 8; i++){
            if (children[i] != nullptr)
                delete children[i];
        }
    }
};

struct OctreeMapLimit
{
    types::Point3D min;
    types::Point3D max;
};

class OctreeMap
{
public:
    OctreeMap()
    {
        m_root = new OctreeNode();
        constexpr double min = std::numeric_limits<double>::min();
        constexpr double max = std::numeric_limits<double>::max();
        m_map_limits.min = types::Point3D(max, max, max);
        m_map_limits.max = types::Point3D(min, min, min);
    }

    ~OctreeMap()
    {
        delete m_root;
    }

    void InsertPointCloud(const std::vector<types::Point3D>& points)
    {
        for (const auto& point : points)
            InsertPoint(point);
    }

    void InsertPoint(const types::Point3D& point)
    {
        InsertPoint(m_root, point);
    }

    types::Point3D GetNearestPoint(const types::Point3D& point)
    {
        return GetNearestPoint(m_root, point);
    }

    void GetNearestPoints(const types::Point3D& point, const double& radius, std::vector<types::Point3D>& points)
    {
        GetNearestPoints(m_root, point, radius, points);
    }

    bool IsValid(const types::Point3D& point)
    {
        return IsInMap(point);
    }
protected:
    void InsertPoint(OctreeNode* node, const types::Point3D& point)
    {
        if (!IsInMap(point))
            return;

        if (IsLeafNode(node))
        {
            node->points.push_back(point);
            UpdateMapLimits(point);
            if (node->points.size() > 10)
                SplitNode(node);
        }
        else
        {
            int index = GetChildIndex(node, point);
            if (node->children[index] == nullptr)
                node->children[index] = new OctreeNode();
            InsertPoint(node->children[index], point);
        }
    }

    bool IsInMap(const types::Point3D& point)
    {
        return (point(0) >= m_map_limits.min(0) && point(0) <= m_map_limits.max(0) &&
                point(1) >= m_map_limits.min(1) && point(1) <= m_map_limits.max(1) &&
                point(2) >= m_map_limits.min(2) && point(2) <= m_map_limits.max(2));
    }

    bool IsLeafNode(OctreeNode* node)
    {
        for (int i = 0; i < 8; i++){
            if (node->children[i] != nullptr)
                return false;
        }
        return true;
    }

    void UpdateMapLimits(const types::Point3D& point)
    {
        if (point(0) < m_map_limits.min(0)) m_map_limits.min(0) = point(0);
        if (point(0) > m_map_limits.max(0)) m_map_limits.max(0) = point(0);
        if (point(1) < m_map_limits.min(1)) m_map_limits.min(1) = point(1);
        if (point(1) > m_map_limits.max(1)) m_map_limits.max(1) = point(1);
        if (point(2) < m_map_limits.min(2)) m_map_limits.min(2) = point(2);
        if (point(2) > m_map_limits.max(2)) m_map_limits.max(2) = point(2);
    }

    int GetChildIndex(OctreeNode* node, const types::Point3D& point)
    {
        int index = 0;
        if (point(0) >= (m_map_limits.min(0) + m_map_limits.max(0)) / 2)
            index += 1;
        if (point(1) >= (m_map_limits.min(1) + m_map_limits.max(1)) / 2)
            index += 2;
        if (point(2) >= (m_map_limits.min(2) + m_map_limits.max(2)) / 2)
            index += 4;
        return index;
    }

    void SplitNode(OctreeNode* node)
    {
        for (uint i = 0; i < node->points.size(); i++)
        {
            int index = GetChildIndex(node, node->points[i]);
            if (node->children[index] == nullptr)
                node->children[index] = new OctreeNode();
            node->children[index]->points.push_back(node->points[i]);
        }
        node->points.clear();
    }

    types::Point3D GetNearestPoint(OctreeNode* node, const types::Point3D& point)
    {
        if (IsLeafNode(node))
        {
            double min_distance = 1000000;
            types::Point3D nearest_point;
            for (uint i = 0; i < node->points.size(); i++)
            {
                double distance = (node->points[i] - point).norm();
                if (distance < min_distance)
                {
                    min_distance = distance;
                    nearest_point = node->points[i];
                }
            }
            return nearest_point;
        }
        else
        {
            int index = GetChildIndex(node, point);
            if (node->children[index] != nullptr)
                return GetNearestPoint(node->children[index], point);
            else
            {
                double min_distance = 1000000;
                types::Point3D nearest_point;
                for (int i = 0; i < 8; i++)
                {
                    if (node->children[i] != nullptr)
                    {
                        types::Point3D temp_point = GetNearestPoint(node->children[i], point);
                        double distance = (temp_point - point).norm();
                        if (distance < min_distance)
                        {
                            min_distance = distance;
                            nearest_point = temp_point;
                        }
                    }
                }
                return nearest_point;
            }
        }
    }

    void GetNearestPoints(OctreeNode* node, const types::Point3D& point, const double& radius, std::vector<types::Point3D>& points)
    {
        if (IsLeafNode(node))
        {
            for (uint i = 0; i < node->points.size(); i++)
            {
                double distance = (node->points[i] - point).norm();
                if (distance < radius)
                    points.push_back(node->points[i]);
            }
        }
        else
        {
            int index = GetChildIndex(node, point);
            if (node->children[index] != nullptr)
                GetNearestPoints(node->children[index], point, radius, points);
            else
            {
                for (int i = 0; i < 8; i++)
                {
                    if (node->children[i] != nullptr)
                        GetNearestPoints(node->children[i], point, radius, points);
                }
            }
        }
    }
protected:
    OctreeMapLimit m_map_limits;
    OctreeNode* m_root;
};

} // namespace front_end
} // namespace point_cloud

#endif // __OCTREE_MAP_H__