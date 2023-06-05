#include "common/tt_test.h"
#include "gps/gps_convertor.h"

JUST_RUN_TEST(gps_convertor, test) 
TEST(gps_convertor, test) 
{
    front_end::GpsConverter gps_converter(Eigen::Vector3d(39.984, 116.306, 1110));
    Eigen::Vector3d lla(39.984, 116.306, 1110);

    Eigen::Vector3d enu;
    gps_converter.LLAToENU(lla, enu);
    std::cout  << "enu: " << enu.transpose() << std::endl; 

    Eigen::Vector3d lla2;
    gps_converter.ENUToLLA(enu, lla2);
    std::cout  << "lla2: " << lla2.transpose() << std::endl;
}
