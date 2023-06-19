#include "common/tt_test.h"
#include "camera/types/depth_map.h"

#include <fstream>
#include <sstream>
#include <iostream>

JUST_RUN_TEST(depth_map, test) 
TEST(depth_map, test) 
{
    // std::cout << "depth_map test" << std::endl;

    std::vector<front_end::point_cloud::DepthMap> depth_maps;
    std::vector<types::Pose3D> poses;

    // 读取深度图
    front_end::point_cloud::DepthMap::Param param;
    std::string img_path = "../../src/test/data/depth/";
    for(int i = 1; i < 6; i++) {
        std::string color_path = img_path + "color/" + std::to_string(i) + ".png";
        std::string depth_path = img_path + "depth/" + std::to_string(i) + ".pgm";
        depth_maps.push_back(front_end::point_cloud::DepthMap(param, color_path, depth_path));
    }

    // 读取相机位姿
    std::ifstream fin(img_path + "pose.txt");
    double data[7];
    std::string line;
    while (std::getline(fin, line)) {
        std::stringstream ss(line);
        for (int i = 0; i < 7; i++) ss >> data[i];
        
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        Eigen::Vector3d t( data[0], data[1], data[2] );

        poses.push_back(types::Pose3D(t, q));

        // std::cout << "pose: " << poses[poses.size() - 1] << std::endl;
    }
    fin.close();

    // 生成点云
    front_end::point_cloud::PointCloudXYZ point_cloud;
    for(uint i = 0; i < depth_maps.size(); i++) {
        std::vector<Eigen::Vector3d> pc = depth_maps[i].GeneratePointCloudXYZ(poses[i]).GetPointCloud();
        point_cloud.AddPoints(pc);
    }
    
    std::cout << "point_cloud size: " << point_cloud.GetPointCloud().size() << std::endl;
}
