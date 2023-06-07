#include "sensor_interface.h"

#include <fstream>
#include <iostream>
#include <sstream>

namespace simulation
{

static void (*g_gps_cb)(const front_end::Gps&) = nullptr;
void RegisterGpsCallback(void(*cb)(const front_end::Gps&)){
    g_gps_cb = cb;
}

static void (*g_imu_cb)(const front_end::Imu&) = nullptr;
void RegisterImuCallback(void(*cb)(const front_end::Imu&)){
    g_imu_cb = cb;
}

void ReadData(const std::string& file_path)
{
    static std::ifstream s_fin;
    static bool s_is_first = false;
    
    if(!s_is_first){
        s_is_first = true;
        s_fin.open(file_path, std::ios::in);

        if(!s_fin.is_open()){
            std::cerr << "Failed to open file: " << file_path << std::endl;
            exit(-1);
        }
    }

    std::string line;
    if(!std::getline(s_fin, line)){
        std::cout << "End of file" << std::endl;
        s_fin.close();
        exit(0);
    }

    std::stringstream ss(line);
    std::string type;
    ss >> type;
    
    if(type == "imu"){
        double time_stamp;
        double acc_x, acc_y, acc_z;
        double gyr_x, gyr_y, gyr_z;
        ss >> time_stamp >> acc_x >> acc_y >> acc_z >> gyr_x >> gyr_y >> gyr_z;
        g_imu_cb((front_end::Imu){time_stamp, Eigen::Vector3d(acc_x, acc_y, acc_z), Eigen::Vector3d(gyr_x, gyr_y, gyr_z)});
    }
    else if(type == "gps"){
        double time_stamp;
        double lat, lon, alt;
        double std_lat, std_lon, std_alt;
        ss >> time_stamp >> lat >> lon >> alt >> std_lat >> std_lon >> std_alt;

        Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
        cov << std_lat, 0, 0, 0, std_lon, 0, 0, 0, std_alt;

        g_gps_cb((front_end::Gps){time_stamp, Eigen::Vector3d(lat, lon, alt), cov});
    }
    else{
        std::cout << "Error: unknown type: " << type << std::endl;
    }
}

} // namespace simulation
