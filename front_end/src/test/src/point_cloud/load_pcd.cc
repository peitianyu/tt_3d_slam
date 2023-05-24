#include"load_pcd.h"


std::vector<types::Point3D> load_pcd(const std::string& file_name)
{
    std::vector<types::Point3D> points;
    std::ifstream file(file_name);
    std::string line;

    while (std::getline(file, line)){ if (line == "DATA ascii") break;}

    while(std::getline(file, line)){
        std::stringstream ss(line);
        double x, y, z;
        ss >> x >> y >> z;
        points.emplace_back(x, y, z);
    }

    ASSERT_EQ(points.size(), 3056);
    return points;
}

